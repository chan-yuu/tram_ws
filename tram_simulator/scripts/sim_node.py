#!/usr/bin/env python3
"""Vehicle dynamics simulation node (ROS Noetic).

Subscribes to /control/* topics, integrates the 6-state dynamics model at
100 Hz, and publishes standard ROS messages for visualization and control.
"""
import numpy as np
import rospy
import tf

from std_msgs.msg import Float64, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import (PoseStamped, TwistStamped,
                                Quaternion, Point)
from std_srvs.srv import Empty, EmptyResponse

from tram_msgs.msg import VehicleState


def _quat_from_yaw(yaw):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class SimNode:
    def __init__(self):
        rospy.init_node('sim_node', anonymous=False)

        # --- load vehicle model ---
        from tram_simulator.vehicle_bridge import build_from_description
        self.vehicle, self.actuators = build_from_description()

        # --- parameters ---
        self.dt = rospy.get_param('~dt', 0.01)
        self.method = rospy.get_param('~integration_method', 'rk4')
        self.hb_timeout = rospy.get_param('~heartbeat_timeout', 0.5)
        self.hb_brake_decel = rospy.get_param('~heartbeat_brake_decel', 1.0)
        trace_interval = rospy.get_param('~trace_interval', 0.05)

        x0 = rospy.get_param('~initial_x', 0.0)
        y0 = rospy.get_param('~initial_y', 0.0)
        psi0 = rospy.get_param('~initial_psi', 0.0)

        # --- state [x, y, psi, vx, vy, r] ---
        self._init_state = np.array([x0, y0, psi0, 0.0, 0.0, 0.0])
        self.q = self._init_state.copy()

        # --- latest control commands ---
        self._tf = 0.0   # torque front  (N·m)
        self._tr = 0.0   # torque rear   (N·m)
        self._bd = 0.0   # brake decel   (m/s²)
        self._df = 0.0   # steering front (rad)
        self._dr = 0.0   # steering rear  (rad)

        self._last_hb = rospy.Time(0)
        self._last_trace = rospy.Time(0)
        self._trace_dt = rospy.Duration(trace_interval)

        # --- subscribers ---
        rospy.Subscriber('/control/torque_front',      Float64, self._cb_tf)
        rospy.Subscriber('/control/torque_rear',       Float64, self._cb_tr)
        rospy.Subscriber('/control/brake_decel',       Float64, self._cb_bd)
        rospy.Subscriber('/control/steering_cmd',      Float64, self._cb_df)
        rospy.Subscriber('/control/steering_rear_cmd', Float64, self._cb_dr)
        rospy.Subscriber('/control/heartbeat',         Header,  self._cb_hb)

        # --- publishers ---
        self.pub_state  = rospy.Publisher('/sim/vehicle_state', VehicleState, queue_size=1)
        self.pub_odom   = rospy.Publisher('/sim/odom',   Odometry,      queue_size=1)
        self.pub_pose   = rospy.Publisher('/sim/pose',   PoseStamped,   queue_size=1)
        self.pub_twist  = rospy.Publisher('/sim/twist',  TwistStamped,  queue_size=1)
        self.pub_trace  = rospy.Publisher('/sim/path_trace', Path,      queue_size=1, latch=False)

        # --- TF broadcaster ---
        self.tf_br = tf.TransformBroadcaster()

        # --- path trace buffer ---
        self.trace_msg = Path()
        self.trace_msg.header.frame_id = 'map'

        # --- reset service ---
        rospy.Service('/sim/reset', Empty, self._srv_reset)

        # --- main timer ---
        rospy.Timer(rospy.Duration(self.dt), self._update)

        rospy.loginfo("[sim_node] started — dt=%.3f s, method=%s", self.dt, self.method)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_tf(self, msg): self._tf = msg.data
    def _cb_tr(self, msg): self._tr = msg.data
    def _cb_bd(self, msg): self._bd = max(msg.data, 0.0)
    def _cb_df(self, msg): self._df = msg.data
    def _cb_dr(self, msg): self._dr = msg.data
    def _cb_hb(self, msg): self._last_hb = rospy.Time.now()

    def _srv_reset(self, _req):
        self.q = self._init_state.copy()
        self.actuators.reset()
        self.trace_msg.poses.clear()
        rospy.loginfo("[sim_node] state reset")
        return EmptyResponse()

    # ------------------------------------------------------------------
    # Update loop
    # ------------------------------------------------------------------
    def _update(self, _event):
        now = rospy.Time.now()

        # Safety: zero commands if heartbeat lost
        if (now - self._last_hb).to_sec() > self.hb_timeout and \
                self._last_hb != rospy.Time(0):
            self._tf = 0.0
            self._tr = 0.0
            self._df = 0.0
            self._dr = 0.0
            self._bd = self.hb_brake_decel

        # Actuator layer
        cmd = np.array([self._df, self._dr, self._tf, self._tr])
        u = self.actuators.step(cmd, float(self.q[3]), self.dt)

        # Optional friction brake as external disturbance
        w = None
        if self._bd > 1e-4:
            w = {'F_x': self.vehicle.m * self._bd}

        # Dynamics integration
        self.q = self.vehicle.step(self.q, u, self.dt, method=self.method, w=w)

        self._publish(now, u)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish(self, now, u):
        x, y, psi, vx, vy, r = self.q
        speed = float(np.hypot(vx, vy))
        quat = _quat_from_yaw(psi)

        # --- VehicleState (custom) ---
        state = VehicleState()
        state.header.stamp = now
        state.header.frame_id = 'map'
        state.x = x; state.y = y; state.psi = psi
        state.v_x = vx; state.v_y = vy; state.r = r
        state.speed = speed
        self.pub_state.publish(state)

        # --- Odometry (standard) ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position = Point(x=x, y=y, z=0.0)
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = r
        self.pub_odom.publish(odom)

        # --- PoseStamped (standard) ---
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pub_pose.publish(pose)

        # --- TwistStamped (standard) ---
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist = odom.twist.twist
        self.pub_twist.publish(twist_msg)

        # --- TF map → base_link ---
        q_arr = tf.transformations.quaternion_from_euler(0.0, 0.0, psi)
        self.tf_br.sendTransform(
            (x, y, 0.0), q_arr, now, 'base_link', 'map')

        # --- Path trace (accumulated) ---
        if (now - self._last_trace) >= self._trace_dt:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'map'
            ps.pose = odom.pose.pose
            self.trace_msg.poses.append(ps)
            self.trace_msg.header.stamp = now
            self.pub_trace.publish(self.trace_msg)
            self._last_trace = now


if __name__ == '__main__':
    node = SimNode()
    rospy.spin()
