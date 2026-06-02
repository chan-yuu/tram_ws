#!/usr/bin/env python3
"""Main control node — PI+LQR or MPC strategy.

Subscribes:
  /sim/vehicle_state       tram_msgs/VehicleState
  /planning/path           tram_msgs/TramPath

Publishes (ROS interface):
  /control/torque_front    std_msgs/Float64
  /control/torque_rear     std_msgs/Float64
  /control/brake_decel     std_msgs/Float64
  /control/steering_cmd    std_msgs/Float64
  /control/steering_rear_cmd std_msgs/Float64
  /control/heartbeat       std_msgs/Header

Debug (rqt_plot):
  /debug/speed_actual /debug/speed_ref /debug/lateral_error
  /debug/heading_error /debug/steering_front /debug/steering_rear
  /debug/accel_cmd /debug/progress /debug/stop_dwell_elapsed
  /debug/stop_dwell_remaining
"""
import json
import os
import numpy as np
import rospy
import rospkg

from std_msgs.msg import Float64, Header, String
from tram_msgs.msg import VehicleState, TramPath, ControlCmd

# controller modules
from tram_control.path_tracker import find_nearest, compute_errors, estimate_progress_s
from tram_control.strategies.pi_lqr.pi_speed   import PISpeedController
from tram_control.strategies.pi_lqr.lqr_lateral import LQRLateralController
from tram_control.strategies.mpc.mpc_controller  import MPCLateralController


def _pub_f64(pub, val):
    pub.publish(Float64(data=float(val)))


def get_vehicle_dyn_param(name: str, default):
    private_name = '~' + name
    global_name = '/vehicle_6state_dyn/' + name
    if rospy.has_param(private_name):
        return rospy.get_param(private_name)
    return rospy.get_param(global_name, default)


def load_vehicle_json():
    try:
        rp = rospkg.RosPack()
        desc_path = rp.get_path('tram_description')
        json_path = os.path.join(desc_path, 'config', 'vehicle.json')
        with open(json_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        rospy.logwarn("[control_node] Failed to load vehicle.json from tram_description: %s", e)
        return None


class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=False)

        self.mode = rospy.get_param('~controller_mode', 'pi_lqr')  # 'pi_lqr' | 'mpc'
        freq = rospy.get_param('~control_freq', 50.0 if self.mode == 'pi_lqr' else 20.0)
        self.dt = 1.0 / freq

        # Load physical params from vehicle.json dynamically
        vj = load_vehicle_json()
        geom = vj.get("geometry", {}) if vj else {}
        mass_inertia = vj.get("mass_inertia", {}) if vj else {}
        drivetrain = vj.get("drivetrain", {}) if vj else {}

        # Fallback to parameter server or hardcoded defaults
        Cf  = get_vehicle_dyn_param('C_alpha_f', 180000.0)
        Cr  = get_vehicle_dyn_param('C_alpha_r', 180000.0)
        m   = rospy.get_param('~mass', float(mass_inertia.get("mass_awk", 11000.0)))
        a   = rospy.get_param('~a_dist', float(geom.get("a_dist", 3.638)))
        b   = rospy.get_param('~b_dist', float(geom.get("b_dist", 3.962)))
        L   = rospy.get_param('~wheelbase', float(geom.get("wheelbase", 7.600)))
        i   = rospy.get_param('~final_drive_ratio', float(drivetrain.get("final_drive_ratio", 16.6)))
        Rw  = rospy.get_param('~wheel_radius', float(geom.get("wheel_radius", 0.402)))
        eta = get_vehicle_dyn_param('eta_drive', 0.92)
        # Reflected rotor inertia raises the effective longitudinal mass —
        # the plant integrates dv/dt = (F - F_res) / m_eff_long, so converting
        # a_cmd via raw m undercommands torque by m_eff_long/m. Without this
        # the vehicle only achieves ~75% of the commanded longitudinal accel.
        motor_inertia = float(mass_inertia.get("motor_inertia", 0.0) or 0.0)
        n_motors = int(get_vehicle_dyn_param('n_drive_motors', 2))
        m_reflect = n_motors * motor_inertia * (i / Rw) ** 2
        m_eff_long = m + m_reflect

        # Calculate yaw inertia Iz using value from JSON if available, else fallback to estimation
        yaw_inertia = mass_inertia.get("yaw_inertia_izz")
        if yaw_inertia is not None:
            Iz = float(yaw_inertia)
        else:
            Iz = m * a * b

        # Torque limits
        self.T_max_f = rospy.get_param('~T_max_drive', 3000.0)
        self.T_max_r = rospy.get_param('~T_max_drive', 3000.0)
        self.T_min_f = rospy.get_param('~T_min_brake', -3000.0)
        self.T_min_r = rospy.get_param('~T_min_brake', -3000.0)
        self.brake_bias_rear = 0.55

        # Drivetrain scales for a_cmd → torque conversion
        self._drive_scale = eta * i / Rw
        self._brake_scale = i / Rw
        self._m = m
        self._m_eff_long = m_eff_long
        self._Rw = Rw
        self._i  = i
        self._eta = eta

        # Resistance feedforward params (must match sim_params.yaml vehicle_6state_dyn block)
        self._c_roll = get_vehicle_dyn_param('c_roll', 0.015)
        self._cd_a   = get_vehicle_dyn_param('cd_a',   6.0)
        self._rho    = get_vehicle_dyn_param('rho',     1.225)

        # --- longitudinal PI ---
        kp = rospy.get_param('~pi_kp', 1.5)
        ki = rospy.get_param('~pi_ki', 0.3)
        max_a = rospy.get_param('~max_accel', 2.0)
        max_d = rospy.get_param('~max_decel', 3.5)
        self.max_decel = max_d
        self.pi = PISpeedController(kp=kp, ki=ki, max_accel=max_a, max_decel=max_d)

        # --- precision stop profile ---
        self.stop_control_distance = rospy.get_param('~stop_control_distance', 90.0)
        self.stop_comfort_decel = rospy.get_param('~stop_comfort_decel', 1.2)
        self.stop_crawl_distance = rospy.get_param('~stop_crawl_distance', 5.0)
        self.stop_crawl_speed = rospy.get_param('~stop_crawl_speed', 0.6)
        self.stop_final_distance = rospy.get_param('~stop_final_distance', 0.8)
        self.stop_final_speed = rospy.get_param('~stop_final_speed', 0.18)
        self.stop_min_crawl_speed = rospy.get_param('~stop_min_crawl_speed', 0.08)
        self.stop_crawl_accel_min = rospy.get_param('~stop_crawl_accel_min', 0.35)
        self.stop_crawl_entry_margin = rospy.get_param('~stop_crawl_entry_margin', 0.75)
        self.stop_tolerance = rospy.get_param('~stop_tolerance', 0.15)
        self.stop_speed_tolerance = rospy.get_param('~stop_speed_tolerance', 0.05)
        self.stop_hold_brake_decel = rospy.get_param('~stop_hold_brake_decel', 0.8)

        # --- curvature speed limiter ---
        self.curve_speed_limit_enable = rospy.get_param('~curve_speed_limit_enable', True)
        self.curve_lateral_accel_limit = rospy.get_param('~curve_lateral_accel_limit', 1.2)
        self.curve_speed_min = rospy.get_param('~curve_speed_min', 2.0)

        # --- speed-reference lookahead ---
        # Distance the speed set-point reads ahead of the nearest path point.
        # At cold start (vehicle and path both at v=0) this is what breaks the
        # PI deadlock; during ramps it provides phase lead.
        self.speed_lookahead_min = rospy.get_param('~speed_lookahead_min', 2.0)
        self.speed_lookahead_time = rospy.get_param('~speed_lookahead_time', 0.3)
        self._curve_speed_limit = float("nan")
        self._curve_speed_limited = False

        # --- low-rate control detail log ---
        self.debug_control_detail_enable = rospy.get_param('~debug_control_detail_enable', True)
        self.debug_control_detail_period = rospy.get_param('~debug_control_detail_period', 1.0)
        self.debug_control_detail_curve_only = rospy.get_param('~debug_control_detail_curve_only', False)
        self.debug_control_detail_kappa_threshold = rospy.get_param('~debug_control_detail_kappa_threshold', 0.005)
        self.debug_control_detail_error_threshold = rospy.get_param('~debug_control_detail_error_threshold', 0.5)

        # --- lateral controller ---
        delta_max = rospy.get_param('~delta_max', 0.2618)
        self.delta_max = float(delta_max)
        if self.mode == 'mpc':
            N  = rospy.get_param('~mpc_horizon', 15)
            dt_mpc = rospy.get_param('~mpc_dt', 0.05)
            q_diag = rospy.get_param('~mpc_Q', None)
            r_diag = rospy.get_param('~mpc_R', None)
            delta_rate_max = rospy.get_param('~delta_rate_max', 0.1)
            self.lateral = MPCLateralController(
                Cf=Cf, Cr=Cr, mass=m, Iz=Iz, a_dist=a, b_dist=b, L=L,
                N=N, dt_mpc=dt_mpc, Q=q_diag, R=r_diag,
                delta_max=delta_max, delta_rate_max=delta_rate_max)
        else:
            q_diag = rospy.get_param('~lqr_Q', None)
            r_diag = rospy.get_param('~lqr_R', None)
            self.lateral = LQRLateralController(
                Cf=Cf, Cr=Cr, mass=m, Iz=Iz, a_dist=a, b_dist=b, L=L,
                Q=q_diag, R=r_diag, dt=self.dt, delta_max=delta_max)

        # --- state ---
        self.path: list = []
        self.nearest_idx: int = 0
        self.state: VehicleState = None
        self._path_done = False
        self._completed_stops = set()
        self._active_stop_idx = None
        self._holding_stop_idx = None
        self._hold_started_at = None
        self._hold_until = rospy.Time(0)
        self._terminal_hold = False
        self._stop_miss_logged = set()
        self._stop_state = "none"
        self._stop_signed_distance = float("nan")
        self._scenario_id = ""

        # --- subscribers ---
        rospy.Subscriber('/sim/vehicle_state', VehicleState, self._cb_state, queue_size=1)
        rospy.Subscriber('/planning/path',     TramPath,     self._cb_path,  queue_size=1)

        # --- publishers: mandatory ROS interface ---
        self.pub_tf   = rospy.Publisher('/control/torque_front',     Float64, queue_size=1)
        self.pub_tr   = rospy.Publisher('/control/torque_rear',      Float64, queue_size=1)
        self.pub_bd   = rospy.Publisher('/control/brake_decel',      Float64, queue_size=1)
        self.pub_df   = rospy.Publisher('/control/steering_cmd',     Float64, queue_size=1)
        self.pub_dr   = rospy.Publisher('/control/steering_rear_cmd', Float64, queue_size=1)
        self.pub_hb   = rospy.Publisher('/control/heartbeat',        Header,  queue_size=1)
        self.pub_echo = rospy.Publisher('/sim/control_echo',         ControlCmd, queue_size=1)

        # --- debug publishers ---
        self.pub_dbg = {
            'speed_actual':  rospy.Publisher('/debug/speed_actual',  Float64, queue_size=1),
            'speed_ref':     rospy.Publisher('/debug/speed_ref',     Float64, queue_size=1),
            'lateral_error': rospy.Publisher('/debug/lateral_error', Float64, queue_size=1),
            'heading_error': rospy.Publisher('/debug/heading_error', Float64, queue_size=1),
            'steering_front':rospy.Publisher('/debug/steering_front',Float64, queue_size=1),
            'steering_rear': rospy.Publisher('/debug/steering_rear', Float64, queue_size=1),
            'accel_cmd':     rospy.Publisher('/debug/accel_cmd',     Float64, queue_size=1),
            'progress':      rospy.Publisher('/debug/progress',      Float64, queue_size=1),
            'stop_distance': rospy.Publisher('/debug/stop_distance', Float64, queue_size=1),
            'stop_abs_error':rospy.Publisher('/debug/stop_abs_error',Float64, queue_size=1),
            'stop_state':    rospy.Publisher('/debug/stop_state',    String,  queue_size=1),
            'stop_dwell_elapsed': rospy.Publisher('/debug/stop_dwell_elapsed', Float64, queue_size=1),
            'stop_dwell_remaining': rospy.Publisher('/debug/stop_dwell_remaining', Float64, queue_size=1),
        }

        # --- heartbeat timer (always running) ---
        rospy.Timer(rospy.Duration(0.1), self._send_heartbeat)
        # --- control loop ---
        rospy.Timer(rospy.Duration(self.dt), self._control_loop)

        rospy.loginfo("[control_node] mode=%s, freq=%.0f Hz", self.mode, freq)

    # ------------------------------------------------------------------
    def _cb_state(self, msg: VehicleState):
        self.state = msg

    def _cb_path(self, msg: TramPath):
        self.path = list(msg.points)
        self.nearest_idx = 0
        self._path_done = False
        self._completed_stops.clear()
        self._active_stop_idx = None
        self._holding_stop_idx = None
        self._hold_started_at = None
        self._hold_until = rospy.Time(0)
        self._terminal_hold = False
        self._stop_miss_logged.clear()
        self._stop_state = "none"
        self._stop_signed_distance = float("nan")
        self._scenario_id = msg.scenario_id
        self.pi.reset()
        if hasattr(self.lateral, 'reset'):
            self.lateral.reset()
        rospy.loginfo("[control_node] new path: %d points, scenario='%s'",
                      len(self.path), msg.scenario_id)

    # ------------------------------------------------------------------
    def _send_heartbeat(self, _event):
        hdr = Header()
        hdr.stamp = rospy.Time.now()
        hdr.frame_id = 'control_node'
        self.pub_hb.publish(hdr)

    # ------------------------------------------------------------------
    def _control_loop(self, _event):
        if self.state is None or not self.path:
            self._publish_zero()
            return
        if self._terminal_hold:
            self._stop_state = "terminal_hold"
            self.pi.reset()
            self._publish_hold()
            self._publish_stop_debug()
            return
        if self._path_done:
            self._publish_zero()
            return

        now = rospy.Time.now()
        s = self.state
        vx = float(s.v_x)
        vy = float(s.v_y)
        r  = float(s.r)

        if self._holding_stop_idx is not None:
            if now < self._hold_until:
                self.pi.reset()
                self._publish_hold()
                self._publish_stop_debug()
                return
            if self._is_terminal_stop(self._holding_stop_idx):
                self._terminal_hold = True
                self._path_done = True
                self._stop_state = "terminal_hold"
                self.pi.reset()
                self._publish_hold()
                self._publish_stop_debug()
                rospy.loginfo_once("[control_node] terminal stop reached; holding at path end")
                return
            self._completed_stops.add(self._holding_stop_idx)
            self.nearest_idx = min(self._holding_stop_idx + 1, len(self.path) - 1)
            self._active_stop_idx = None
            self._holding_stop_idx = None
            self._hold_started_at = None

        # --- nearest path point ---
        min_idx = max(self._completed_stops) + 1 if self._completed_stops else 0
        self.nearest_idx, dist = find_nearest(
            self.path, s.x, s.y, max(self.nearest_idx, min_idx))
        self.nearest_idx = max(self.nearest_idx, min_idx)

        stop_idx = self._active_stop_idx
        if stop_idx is None or stop_idx in self._completed_stops:
            stop_idx = self._next_stop_idx(self.nearest_idx)

        if self.nearest_idx >= len(self.path) - 1 and stop_idx is None:
            self._path_done = True
            self._publish_zero()
            rospy.loginfo_once("[control_node] path complete")
            return

        ref = self.path[self.nearest_idx]
        e_y, e_psi = compute_errors(self.path, self.nearest_idx, s.x, s.y, s.psi)
        current_s = estimate_progress_s(self.path, self.nearest_idx, s.x, s.y)
        kappa = float(ref.kappa)
        # Lookahead speed reference. The lateral targets (yaw/kappa) stay at
        # the nearest point, but for vx we peek a short distance ahead so the
        # PI has a non-zero set-point at cold start (where vehicle and path
        # both sit at v=0 at idx=0) and gets phase lead on accel ramps.
        speed_ref_pt = self._lookahead_speed_point(current_s, vx)
        v_ref = self._stop_aware_speed_ref(speed_ref_pt, stop_idx, current_s, vx, now)
        v_ref = self._curve_limited_speed_ref(v_ref, kappa)

        # --- longitudinal PI ---
        a_cmd = self.pi.compute(v_ref, vx, self.dt)
        a_cmd = self._prevent_stop_profile_underspeed(a_cmd, stop_idx, current_s, vx, v_ref)
        a_cmd = self._unstick_crawl_if_needed(a_cmd, stop_idx, current_s, vx, v_ref)
        a_cmd = self._enforce_stop_decel(a_cmd, stop_idx, current_s, vx, v_ref)
        Tf, Tr, bd = self._accel_to_torques(a_cmd, vx)
        self._log_stop_debug(vx, v_ref, a_cmd)

        # --- lateral (LQR or MPC) ---
        steer = self.lateral.compute(e_y, e_psi, vy, r, vx, kappa)
        delta_f = float(steer[0])
        delta_r = float(steer[1])

        # --- publish control interface ---
        self._publish_command(Tf, Tr, bd, delta_f, delta_r)

        # --- debug ---
        progress = self.nearest_idx / max(len(self.path) - 1, 1)
        _pub_f64(self.pub_dbg['speed_actual'],   vx)
        _pub_f64(self.pub_dbg['speed_ref'],       v_ref)
        _pub_f64(self.pub_dbg['lateral_error'],   e_y)
        _pub_f64(self.pub_dbg['heading_error'],   e_psi)
        _pub_f64(self.pub_dbg['steering_front'],  delta_f)
        _pub_f64(self.pub_dbg['steering_rear'],   delta_r)
        _pub_f64(self.pub_dbg['accel_cmd'],       a_cmd)
        _pub_f64(self.pub_dbg['progress'],        progress)
        self._publish_stop_debug()
        self._log_control_detail(
            ref, current_s, dist, e_y, e_psi, vx, vy, r,
            v_ref, a_cmd, Tf, Tr, bd, delta_f, delta_r,
            kappa, progress,
        )

    # ------------------------------------------------------------------
    def _resist_force(self, vx: float) -> float:
        """Rolling + aero resistance on flat road (mirrors vehicle_6state_dyn.resist_force)."""
        v = max(vx, 0.0)
        f_roll = self._c_roll * self._m * 9.80665 * np.tanh(v / 0.3)
        f_aero = 0.5 * self._rho * self._cd_a * v * v
        return f_roll + f_aero

    def _accel_to_torques(self, a_cmd: float, vx: float):
        """Convert acceleration command to (Tf, Tr, brake_decel) with resistance feedforward."""
        # Use the longitudinal effective mass (raw mass + reflected rotor inertia)
        # so the commanded force actually produces the requested a_cmd at the wheels.
        m_eff = self._m_eff_long
        # Total wheel force = net demand + resistance compensation
        F_total = m_eff * a_cmd + self._resist_force(vx)
        if F_total >= 0.0:
            scale = self._drive_scale
            Tf = float(np.clip(0.5 * F_total / scale, 0, self.T_max_f))
            Tr = float(np.clip(0.5 * F_total / scale, 0, self.T_max_r))
            bd = 0.0
        else:
            scale = self._brake_scale
            bf = 1.0 - self.brake_bias_rear
            br = self.brake_bias_rear
            Tf = float(np.clip(bf * F_total / scale, self.T_min_f, 0))
            Tr = float(np.clip(br * F_total / scale, self.T_min_r, 0))
            motor_force = scale * (Tf + Tr)
            bd = max((motor_force - F_total) / m_eff, 0.0)
        return Tf, Tr, bd

    def _next_stop_idx(self, start_idx: int):
        for i in range(start_idx, len(self.path)):
            if getattr(self.path[i], 'stop_point', False) and i not in self._completed_stops:
                return i
        return None

    def _is_terminal_stop(self, stop_idx: int) -> bool:
        return bool(self.path and stop_idx >= len(self.path) - 1)

    def _stop_aware_speed_ref(self, ref, stop_idx, current_s: float,
                              vx: float, now: rospy.Time) -> float:
        base_v = float(ref.v)
        if stop_idx is None:
            self._stop_state = "none"
            self._stop_signed_distance = float("nan")
            return base_v

        stop_pt = self.path[stop_idx]
        signed_remaining = float(stop_pt.s - current_s)
        abs_error = abs(signed_remaining)
        remaining = max(signed_remaining, 0.0)
        speed = abs(vx)
        self._stop_signed_distance = signed_remaining

        if remaining <= self.stop_control_distance:
            self._active_stop_idx = stop_idx

        if signed_remaining < -self.stop_tolerance and stop_idx not in self._stop_miss_logged:
            self._stop_state = "overshot"
            rospy.logwarn("[control_node] stop point overshot idx=%d, error_s=%.3f m",
                          stop_idx, signed_remaining)
            self._stop_miss_logged.add(stop_idx)

        if abs_error <= self.stop_tolerance and speed <= self.stop_speed_tolerance:
            self._stop_state = "terminal_hold" if self._is_terminal_stop(stop_idx) else "hold"
            dwell = max(float(getattr(stop_pt, 'dwell_time', 0.0)), 0.0)
            self._holding_stop_idx = stop_idx
            self._hold_started_at = now
            self._hold_until = now + rospy.Duration(dwell)
            if self._is_terminal_stop(stop_idx):
                rospy.loginfo("[control_node] terminal stop reached idx=%d, error_s=%.3f m",
                              stop_idx, signed_remaining)
            else:
                rospy.loginfo("[control_node] stop reached idx=%d, error_s=%.3f m, dwell=%.1f s",
                              stop_idx, signed_remaining, dwell)
            return 0.0

        if signed_remaining > self.stop_control_distance:
            self._stop_state = "cruise"
            return base_v

        if signed_remaining > self.stop_crawl_distance:
            self._stop_state = "brake"
            v_limit = np.sqrt(max(2.0 * self.stop_comfort_decel *
                                  (signed_remaining - self.stop_crawl_distance), 0.0))
            v_limit = max(v_limit, self.stop_crawl_speed)
            return min(base_v, float(v_limit))
        elif signed_remaining > self.stop_tolerance:
            if signed_remaining > self.stop_final_distance:
                self._stop_state = "crawl"
                v_limit = self.stop_crawl_speed
            else:
                self._stop_state = "settle"
                ratio = (signed_remaining - self.stop_tolerance) / max(
                    self.stop_final_distance - self.stop_tolerance, 1e-6)
                v_limit = max(self.stop_final_speed * ratio, self.stop_min_crawl_speed)
            return float(v_limit)
        else:
            self._stop_state = "settle"
            v_limit = 0.0

        return float(v_limit)

    def _lookahead_speed_point(self, current_s: float, vx: float):
        """Return a path point a short distance ahead of ``current_s``.

        The returned point's ``v`` is what the speed PI tracks; ``ref.yaw`` /
        ``ref.kappa`` are still taken from the nearest point in the main loop.
        """
        lookahead = max(self.speed_lookahead_min,
                        self.speed_lookahead_time * abs(vx))
        target_s = current_s + lookahead
        n = len(self.path)
        i = self.nearest_idx
        # Linear forward scan from the current nearest point; the path is
        # monotonic in s so this is O(lookahead/ds) per cycle.
        while i < n - 1 and self.path[i].s < target_s:
            i += 1
        return self.path[i]

    def _curve_limited_speed_ref(self, v_ref: float, kappa: float) -> float:
        self._curve_speed_limit = float("nan")
        self._curve_speed_limited = False
        if not self.curve_speed_limit_enable:
            return v_ref
        abs_kappa = abs(float(kappa))
        if abs_kappa < 1e-4 or v_ref <= 0.0:
            return v_ref
        v_limit = np.sqrt(max(self.curve_lateral_accel_limit / abs_kappa, 0.0))
        v_limit = max(float(v_limit), self.curve_speed_min)
        self._curve_speed_limit = v_limit
        self._curve_speed_limited = v_ref > v_limit + 1e-6
        return min(v_ref, v_limit)

    def _unstick_crawl_if_needed(self, a_cmd: float, stop_idx, current_s: float,
                                 vx: float, v_ref: float) -> float:
        if stop_idx is None:
            return a_cmd
        signed_remaining = float(self.path[stop_idx].s - current_s)
        crawl_entry = self.stop_crawl_distance + self.stop_crawl_entry_margin
        stopped_short = self.stop_tolerance < signed_remaining <= crawl_entry
        if stopped_short and vx <= self.stop_speed_tolerance and v_ref > self.stop_speed_tolerance:
            self.pi.reset()
            a_cmd = max(a_cmd, self.stop_crawl_accel_min)
            rospy.logwarn_throttle(
                1.0,
                "[control_node] crawl unstick: stop_distance=%.3f m, v_ref=%.3f m/s, vx=%.3f m/s, a_cmd=%.3f m/s^2",
                signed_remaining, v_ref, vx, a_cmd,
            )
        return a_cmd

    def _prevent_stop_profile_underspeed(self, a_cmd: float, stop_idx,
                                         current_s: float, vx: float,
                                         v_ref: float) -> float:
        if stop_idx is None:
            return a_cmd
        signed_remaining = float(self.path[stop_idx].s - current_s)
        in_stop_profile = signed_remaining <= self.stop_control_distance
        before_final_settle = signed_remaining > self.stop_final_distance
        below_profile = v_ref > vx + self.stop_speed_tolerance
        if not (in_stop_profile and before_final_settle and below_profile and a_cmd < 0.0):
            return a_cmd

        self.pi.reset()
        if signed_remaining <= self.stop_crawl_distance + self.stop_crawl_entry_margin:
            a_cmd = self.stop_crawl_accel_min if vx <= self.stop_speed_tolerance else 0.0
        else:
            a_cmd = 0.0
        rospy.logwarn_throttle(
            1.0,
            "[control_node] stop profile underspeed guard: stop_distance=%.3f m, v_ref=%.3f m/s, vx=%.3f m/s, a_cmd=%.3f m/s^2",
            signed_remaining, v_ref, vx, a_cmd,
        )
        return a_cmd

    def _enforce_stop_decel(self, a_cmd: float, stop_idx, current_s: float,
                            vx: float, v_ref: float) -> float:
        if stop_idx is None:
            return a_cmd
        stop_s = float(self.path[stop_idx].s)
        signed_remaining = stop_s - current_s
        remaining = max(signed_remaining - self.stop_tolerance, 0.05)
        if signed_remaining > self.stop_control_distance:
            return a_cmd

        if signed_remaining > self.stop_final_distance and v_ref > vx + self.stop_speed_tolerance:
            return max(a_cmd, 0.0)

        # In crawl mode, allow positive acceleration if the vehicle stopped
        # short of the stop point. Otherwise it can deadlock at v_ref > 0.
        if a_cmd > 0.0 and v_ref > vx:
            return a_cmd

        v = max(vx, 0.0)
        required = -(v * v) / (2.0 * remaining)
        required = max(required, -self.max_decel)
        if remaining <= self.stop_crawl_distance or required < -self.stop_comfort_decel:
            return min(a_cmd, required)
        return a_cmd

    def _publish_stop_debug(self):
        dist = self._stop_signed_distance
        _pub_f64(self.pub_dbg['stop_distance'], dist)
        _pub_f64(self.pub_dbg['stop_abs_error'], abs(dist) if np.isfinite(dist) else float("nan"))
        self.pub_dbg['stop_state'].publish(String(data=self._stop_state))
        dwell_elapsed, dwell_remaining = self._dwell_debug_values(rospy.Time.now())
        _pub_f64(self.pub_dbg['stop_dwell_elapsed'], dwell_elapsed)
        _pub_f64(self.pub_dbg['stop_dwell_remaining'], dwell_remaining)

    def _dwell_debug_values(self, now: rospy.Time):
        if self._holding_stop_idx is None or self._hold_started_at is None:
            return float("nan"), float("nan")
        elapsed = max((now - self._hold_started_at).to_sec(), 0.0)
        if self._is_terminal_stop(self._holding_stop_idx):
            return elapsed, 0.0
        remaining = max((self._hold_until - now).to_sec(), 0.0)
        return elapsed, remaining

    def _log_stop_debug(self, vx: float, v_ref: float, a_cmd: float):
        if not np.isfinite(self._stop_signed_distance):
            return
        if abs(self._stop_signed_distance) <= self.stop_control_distance:
            rospy.loginfo_throttle(
                1.0,
                "[control_node] stop_state=%s stop_distance=%.3f m vx=%.3f m/s v_ref=%.3f m/s a_cmd=%.3f m/s^2",
                self._stop_state, self._stop_signed_distance, vx, v_ref, a_cmd,
            )

    def _log_control_detail(self, ref, current_s: float, nearest_dist: float,
                            e_y: float, e_psi: float, vx: float, vy: float,
                            yaw_rate: float, v_ref: float, a_cmd: float,
                            Tf: float, Tr: float, brake_decel: float,
                            delta_f: float, delta_r: float,
                            kappa: float, progress: float):
        if not self.debug_control_detail_enable:
            return

        curve_active = abs(kappa) >= self.debug_control_detail_kappa_threshold
        error_active = (
            abs(e_y) >= self.debug_control_detail_error_threshold or
            abs(e_psi) >= np.deg2rad(5.0)
        )
        if self.debug_control_detail_curve_only and not (
                curve_active or error_active or self._curve_speed_limited):
            return

        delta_limit = max(self.delta_max, 1e-6)
        sat_f = abs(delta_f) >= 0.95 * delta_limit
        sat_r = abs(delta_r) >= 0.95 * delta_limit
        beta = np.arctan2(vy, max(abs(vx), 0.1))
        ay_ref = v_ref * v_ref * kappa
        yaw_rate_ref = v_ref * kappa
        curve_limit = self._curve_speed_limit if np.isfinite(self._curve_speed_limit) else -1.0
        stop_dist = self._stop_signed_distance if np.isfinite(self._stop_signed_distance) else 9999.0

        rospy.loginfo_throttle(
            self.debug_control_detail_period,
            "[control_node][detail] scenario=%s mode=%s idx=%d/%d progress=%.1f%% nearest_dist=%.2f m s=%.2f ref_s=%.2f\n"
            "  pose: x=%.2f y=%.2f psi=%.1f deg ref_yaw=%.1f deg kappa=%.4f 1/m\n"
            "  speed: vx=%.2f vy=%.2f beta=%.1f deg r=%.3f rad/s v_ref=%.2f curve_limit=%.2f limited=%s\n"
            "  curve: ay_ref=%.2f m/s^2 yaw_rate_ref=%.3f rad/s\n"
            "  error: e_y=%.2f m e_psi=%.1f deg\n"
            "  cmd: a=%.2f m/s^2 Tf=%.1f Tr=%.1f brake=%.2f delta_f=%.1f deg delta_r=%.1f deg sat_f=%s sat_r=%s\n"
            "  stop: state=%s distance=%.2f m active_stop=%s",
            self._scenario_id, self.mode, self.nearest_idx, max(len(self.path) - 1, 0),
            progress * 100.0, nearest_dist, current_s, float(ref.s),
            float(self.state.x), float(self.state.y),
            np.rad2deg(float(self.state.psi)), np.rad2deg(float(ref.yaw)), kappa,
            vx, vy, np.rad2deg(beta), yaw_rate, v_ref, curve_limit, self._curve_speed_limited,
            ay_ref, yaw_rate_ref,
            e_y, np.rad2deg(e_psi),
            a_cmd, Tf, Tr, brake_decel, np.rad2deg(delta_f), np.rad2deg(delta_r), sat_f, sat_r,
            self._stop_state, stop_dist, self._active_stop_idx,
        )

    def _publish_command(self, Tf, Tr, bd, delta_f, delta_r):
        _pub_f64(self.pub_tf, Tf)
        _pub_f64(self.pub_tr, Tr)
        _pub_f64(self.pub_bd, bd)
        _pub_f64(self.pub_df, delta_f)
        _pub_f64(self.pub_dr, delta_r)

        echo = ControlCmd()
        echo.header.stamp = rospy.Time.now()
        echo.torque_front   = Tf
        echo.torque_rear    = Tr
        echo.brake_decel    = bd
        echo.steering_front = delta_f
        echo.steering_rear  = delta_r
        echo.controller_mode = self.mode
        self.pub_echo.publish(echo)

    def _publish_hold(self):
        self._publish_command(0.0, 0.0, self.stop_hold_brake_decel, 0.0, 0.0)

    def _publish_zero(self):
        for pub in [self.pub_tf, self.pub_tr, self.pub_bd,
                    self.pub_df, self.pub_dr]:
            pub.publish(Float64(data=0.0))


if __name__ == '__main__':
    node = ControlNode()
    rospy.spin()
