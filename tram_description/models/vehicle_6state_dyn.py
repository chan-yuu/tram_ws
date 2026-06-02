"""
六状态横纵向耦合动力学模型（Model IV，修订版 v2）。

状态（质心 CG）：q = [x, y, psi, v_x, v_y, r]
控制（plant 输入，路面轮转角 + 电机端扭矩）：u = [delta_f, delta_r, T_f, T_r]

v2 相对 v1 的改动：
  A. 偏航转动惯量 I_z 为 None 时按 m*a*b 自动估算（不再硬编码 63000）。
  B. 电机折算惯量计入【纵向】有效质量 m_eff = m + n*I_m*(i/R_w)^2，
     避免低估纵向加速所需的力（横向/横摆仍用 m、I_z）。
  C. build 中轴距优先取 geometry.wheelbase；坡度区分“工况坡度”与
     “最大坡度限值”，max_slope_percent 仅作限值，不当作常值工况。
  D. 新增 VehicleActuators：转向比(46.7)+速率/转角限幅，电机扭矩
     一阶迟滞+变化率限幅+(可选)功率限扭，作为 plant 之外的执行器层。

控制完整性：加速 (T>0)、减速/制动 (T<0)、前后转向 (delta_f/delta_r)；
运动学约束：不允许反向，v_x 最多减到 0（到 0 即驻车，v_y、r 归零）。
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Optional, Tuple

G = 9.80665
EPS_VX = 0.1        # 滑移角分母下限，防止除零
V_ROLL_EPS = 0.3    # 滚阻平滑速度尺度（m/s），静止时滚阻 -> 0
V_LAT_EPS = 1.0     # 侧向力淡入速度尺度（m/s），低速去奇异


def _sat(value: float, limit: float) -> float:
    return float(np.clip(value, -limit, limit))


class FourWSVehicle6StateDyn:
    def __init__(
        self,
        wheelbase: float = 7.6,
        a_dist: float = 3.638,
        b_dist: float = 3.962,
        mass: float = 11000.0,
        I_z: Optional[float] = None,       # None -> 按 m*a*b 自动估算
        C_alpha_f: float = 180000.0,
        C_alpha_r: float = 180000.0,
        mu0: float = 0.85,
        c_roll: float = 0.015,
        cd_a: float = 6.0,
        rho: float = 1.225,
        grade_percent: float = 0.0,        # 工况坡度（默认平路）
        wheel_radius: float = 0.402,
        final_drive_ratio: float = 16.6,
        eta_drive: float = 0.92,
        h_cg: float = 1.1,
        motor_inertia: float = 0.0,        # 单台电机转子惯量 (kg·m²)
        n_drive_motors: int = 2,           # 驱动电机数（前+后=2）
        max_slope_percent: float = 12.0,   # 最大坡度限值（仅作约束参考）
    ):
        self.L = wheelbase
        self.a = a_dist
        self.b = b_dist
        self.m = mass
        self.C_alpha_f = C_alpha_f
        self.C_alpha_r = C_alpha_r
        self.mu0 = mu0
        self.c_roll = c_roll
        self.cd_a = cd_a
        self.rho = rho
        self.grade = np.arctan(grade_percent / 100.0)
        self.max_grade = np.arctan(max_slope_percent / 100.0)
        self.R_w = wheel_radius
        self.i_final = final_drive_ratio
        self.eta = eta_drive
        self.h_cg = h_cg
        self.motor_inertia = motor_inertia
        self.n_drive_motors = n_drive_motors

        # 偏航转动惯量：缺省按薄板/集中近似 m*a*b
        self.I_z = float(I_z) if I_z else self.m * self.a * self.b

        # 纵向有效质量：计入折算到车轮的电机转子惯量
        m_reflect = self.n_drive_motors * self.motor_inertia * (self.i_final / self.R_w) ** 2
        self.m_eff_long = self.m + m_reflect

        # 静态轴荷
        self.F_zf = self.m * G * self.b / self.L
        self.F_zr = self.m * G * self.a / self.L

    # ------------------------------------------------------------------
    # 传动标度（驱动与制动分开）
    # ------------------------------------------------------------------

    def _drive_scale(self) -> float:
        return self.eta * self.i_final / self.R_w

    def _brake_scale(self) -> float:
        return self.i_final / self.R_w

    # ------------------------------------------------------------------
    # 坐标变换：CG ↔ 后轴
    # ------------------------------------------------------------------

    def cg_to_rear(self, x: float, y: float, psi: float) -> Tuple[float, float]:
        return x - self.b * np.cos(psi), y - self.b * np.sin(psi)

    def rear_to_cg(self, x_r: float, y_r: float, psi: float) -> Tuple[float, float]:
        return x_r + self.b * np.cos(psi), y_r + self.b * np.sin(psi)

    # ------------------------------------------------------------------
    # 轮胎、轴荷与阻力
    # ------------------------------------------------------------------

    def slip_angles(self, v_x: float, v_y: float, r: float, delta_f: float, delta_r: float):
        vx = max(v_x, EPS_VX)
        alpha_f = delta_f - np.arctan2(v_y + self.a * r, vx)
        alpha_r = delta_r - np.arctan2(v_y - self.b * r, vx)
        return alpha_f, alpha_r

    def axle_loads(self, a_x: float) -> Tuple[float, float]:
        """纵向载荷转移（转移的是整车重量，用 self.m；a_x 为真实纵向加速度）。"""
        dW = self.m * a_x * self.h_cg / self.L
        F_zf = max(self.F_zf - dW, 1.0)
        F_zr = max(self.F_zr + dW, 1.0)
        return F_zf, F_zr

    def combined_axle_forces(self, alpha, Fx_demand, C_alpha, mu, Fz):
        """摩擦椭圆：纵向力先按 mu*Fz 限幅，再用剩余抓地限侧向力。"""
        F_max = mu * Fz
        Fx = float(np.clip(Fx_demand, -F_max, F_max))
        Fy_budget = float(np.sqrt(max(F_max * F_max - Fx * Fx, 0.0)))
        Fy = float(np.clip(C_alpha * alpha, -Fy_budget, Fy_budget))
        return Fx, Fy

    def lateral_forces(self, alpha_f, alpha_r, mu):
        """[遗留接口] 纯侧向、无联合滑移耦合，仅供兼容旧调用。"""
        F_yf = _sat(self.C_alpha_f * alpha_f, mu * self.F_zf)
        F_yr = _sat(self.C_alpha_r * alpha_r, mu * self.F_zr)
        return F_yf, F_yr

    def motor_forces(self, T_f: float, T_r: float) -> Tuple[float, float]:
        def force_of(T: float) -> float:
            scale = self._drive_scale() if T >= 0.0 else self._brake_scale()
            return scale * T
        return force_of(T_f), force_of(T_r)

    def resist_force(self, v_x: float, gamma: float, d_Fx: float = 0.0) -> float:
        v = max(v_x, 0.0)
        roll_scale = float(np.tanh(v / V_ROLL_EPS))  # 静止 -> 0，避免蠕动
        f_roll = self.c_roll * self.m * G * np.cos(gamma) * roll_scale
        f_aero = 0.5 * self.rho * self.cd_a * v * v
        f_grade = self.m * G * np.sin(gamma)
        return f_roll + f_aero + f_grade + d_Fx

    def cruise_torques(self, v_x: float, gamma: Optional[float] = None) -> Tuple[float, float]:
        gamma = self.grade if gamma is None else gamma
        F_res = self.resist_force(v_x, gamma)
        T_each = 0.5 * F_res / self._drive_scale()
        return T_each, T_each

    def force_to_torques(self, F_x_total, T_max_f, T_max_r, T_min_f, T_min_r, brake_bias_rear=0.55):
        """总纵向力 → 前/后电机扭矩（T<0 为制动）。"""
        if F_x_total >= 0:
            scale = self._drive_scale()
            T_f = 0.5 * F_x_total / scale
            T_r = 0.5 * F_x_total / scale
        else:
            scale = self._brake_scale()
            bf = 1.0 - brake_bias_rear
            br = brake_bias_rear
            T_f = bf * F_x_total / scale
            T_r = br * F_x_total / scale
        T_f = float(np.clip(T_f, T_min_f, T_max_f))
        T_r = float(np.clip(T_r, T_min_r, T_max_r))
        return T_f, T_r

    def torques_from_accel_cmd(self, a_cmd, v_x, T_max_f, T_max_r, T_min_f, T_min_r,
                               brake_bias_rear=0.55, gamma=None):
        gamma = self.grade if gamma is None else gamma
        F_res = self.resist_force(max(v_x, 0.0), gamma)
        F_x = self.m * a_cmd + F_res   # 期望加速度 -> 需求力（用真实质量做指令换算）
        return self.force_to_torques(F_x, T_max_f, T_max_r, T_min_f, T_min_r, brake_bias_rear)

    def tire_outputs(self, q, u, w=None) -> dict:
        _, _, _, v_x, v_y, r = q
        delta_f, delta_r, T_f, T_r = u
        w = w or {}
        mu = self.mu0 * (1.0 + w.get("mu", 0.0))
        gamma = self.grade + w.get("gamma", 0.0)
        df = delta_f + w.get("delta_bias", 0.0)
        dr = delta_r

        F_xf_dem, F_xr_dem = self.motor_forces(T_f, T_r)
        F_res = self.resist_force(v_x, gamma, w.get("F_x", 0.0))
        a_x_est = (F_xf_dem + F_xr_dem - F_res) / self.m_eff_long
        F_zf, F_zr = self.axle_loads(a_x_est)

        alpha_f, alpha_r = self.slip_angles(v_x, v_y, r, df, dr)
        F_xf, F_yf = self.combined_axle_forces(alpha_f, F_xf_dem, self.C_alpha_f, mu, F_zf)
        F_xr, F_yr = self.combined_axle_forces(alpha_r, F_xr_dem, self.C_alpha_r, mu, F_zr)
        lat_scale = float(np.clip(v_x / V_LAT_EPS, 0.0, 1.0))
        F_yf *= lat_scale
        F_yr *= lat_scale
        return {"alpha_f": alpha_f, "alpha_r": alpha_r, "F_yf": F_yf, "F_yr": F_yr,
                "F_xf": F_xf, "F_xr": F_xr, "F_zf": F_zf, "F_zr": F_zr,
                "mu": mu, "gamma": gamma, "a_x_est": a_x_est}

    # ------------------------------------------------------------------
    # 状态方程
    # ------------------------------------------------------------------

    def f(self, q, u, w=None):
        x, y, psi, v_x, v_y, r = q
        delta_f, delta_r, T_f, T_r = u
        w = w or {}

        d_Fy = w.get("F_y", 0.0)
        d_Mz = w.get("M_z", 0.0)
        d_Fx = w.get("F_x", 0.0)
        mu = self.mu0 * (1.0 + w.get("mu", 0.0))
        gamma = self.grade + w.get("gamma", 0.0)
        df = delta_f + w.get("delta_bias", 0.0)
        dr = delta_r

        F_xf_dem, F_xr_dem = self.motor_forces(T_f, T_r)
        F_res = self.resist_force(v_x, gamma, d_Fx)

        # 真实纵向加速度估计（计入电机折算惯量）-> 载荷转移
        a_x_est = (F_xf_dem + F_xr_dem - F_res) / self.m_eff_long
        F_zf, F_zr = self.axle_loads(a_x_est)

        alpha_f, alpha_r = self.slip_angles(v_x, v_y, r, df, dr)
        F_xf, F_yf = self.combined_axle_forces(alpha_f, F_xf_dem, self.C_alpha_f, mu, F_zf)
        F_xr, F_yr = self.combined_axle_forces(alpha_r, F_xr_dem, self.C_alpha_r, mu, F_zr)

        lat_scale = float(np.clip(v_x / V_LAT_EPS, 0.0, 1.0))
        F_yf *= lat_scale
        F_yr *= lat_scale

        cf, sf = np.cos(df), np.sin(df)
        cr, sr = np.cos(dr), np.sin(dr)
        Fx_body = (F_xf * cf - F_yf * sf) + (F_xr * cr - F_yr * sr) - F_res
        Fy_body = (F_xf * sf + F_yf * cf) + (F_xr * sr + F_yr * cr) + d_Fy
        Mz = self.a * (F_xf * sf + F_yf * cf) - self.b * (F_xr * sr + F_yr * cr) + d_Mz

        v_x_dot = v_y * r + Fx_body / self.m_eff_long   # 纵向用有效质量
        v_y_dot = -v_x * r + Fy_body / self.m           # 横向用整车质量
        r_dot = Mz / self.I_z

        xdot = v_x * np.cos(psi) - v_y * np.sin(psi)
        ydot = v_x * np.sin(psi) + v_y * np.cos(psi)
        psidot = r
        return np.array([xdot, ydot, psidot, v_x_dot, v_y_dot, r_dot])

    # ------------------------------------------------------------------
    # 运动学约束：不允许反向行驶（v_x 最多到 0）
    # ------------------------------------------------------------------

    def _apply_constraints(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).copy()
        if q[3] <= 0.0:
            q[3] = 0.0
            q[4] = 0.0
            q[5] = 0.0
        return q

    def step_euler(self, q, u, dt, w=None):
        return self._apply_constraints(q + dt * self.f(q, u, w))

    def step_rk4(self, q, u, dt, w=None):
        k1 = self.f(q, u, w)
        k2 = self.f(q + 0.5 * dt * k1, u, w)
        k3 = self.f(q + 0.5 * dt * k2, u, w)
        k4 = self.f(q + dt * k3, u, w)
        return self._apply_constraints(q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4))

    def step(self, q, u, dt, method="rk4", w=None):
        if method == "rk4":
            return self.step_rk4(q, u, dt, w)
        if method == "euler":
            return self.step_euler(q, u, dt, w)
        raise ValueError(f"Unknown integration method: {method}")

    def speed(self, q):
        _, _, _, v_x, v_y, _ = q
        return float(np.hypot(v_x, v_y))

    def linearize(self, q, u, w=None, eps=1e-5):
        """数值雅可比 A=∂f/∂q, B=∂f/∂u（中心差分）。x、y 两列恒为零。"""
        n, m = 6, 4
        A = np.zeros((n, n))
        B = np.zeros((n, m))
        for i in range(n):
            dq = np.zeros(n)
            dq[i] = eps
            A[:, i] = (self.f(q + dq, u, w) - self.f(q - dq, u, w)) / (2 * eps)
        for j in range(m):
            du = np.zeros(m)
            du[j] = eps
            B[:, j] = (self.f(q, u + du, w) - self.f(q, u - du, w)) / (2 * eps)
        return A, B


# ======================================================================
# 执行器层：转向 + 电机扭矩（plant 之外）
# ======================================================================

class VehicleActuators:
    """把控制器的期望指令转成可实现的 plant 输入。

    cmd = [delta_f_cmd, delta_r_cmd, T_f_cmd, T_r_cmd]
      delta_*_cmd：期望【路面轮】转角 (rad)
      T_*_cmd：    期望【电机端】扭矩 (N·m)，<0 为制动/再生

    转向：输入轴速率上限经转向比换算到路面轮，做速率限幅 + 转角限幅。
    电机：扭矩一阶迟滞 + 变化率限幅 +（可选）峰值扭矩/功率限扭。
    标注“占位”的默认值需用实测值替换（JSON 中对应项当前为 null）。
    """

    def __init__(
        self,
        steer_ratio: float = 46.7,
        steer_rate_limit_input_deg_s: float = 520.0,
        steer_angle_max_front_deg: float = 15.0,   # 占位：JSON 为 null，待标定/由最小半径反推
        steer_angle_max_rear_deg: float = 15.0,    # 占位
        torque_tau: float = 0.04,                  # 占位：扭矩一阶迟滞 (s)，待用实测扭矩延迟替换
        torque_rate_max: float = 5000.0,           # 占位：扭矩变化率上限 (N·m/s)
        T_max_drive: float = float("inf"),         # 占位：峰值驱动扭矩 (N·m)
        T_max_brake: float = float("inf"),         # 占位：峰值制动扭矩幅值 (N·m)
        P_max: float = float("inf"),               # 占位：峰值功率 (W)，高速恒功率限扭
    ):
        self.steer_ratio = steer_ratio
        self.steer_rate_rw = np.deg2rad(steer_rate_limit_input_deg_s / steer_ratio)  # rad/s @路面轮
        self.ang_max_f = np.deg2rad(steer_angle_max_front_deg)
        self.ang_max_r = np.deg2rad(steer_angle_max_rear_deg)
        self.torque_tau = torque_tau
        self.torque_rate_max = torque_rate_max
        self.T_max_drive = T_max_drive
        self.T_max_brake = T_max_brake
        self.P_max = P_max
        self.i_final = None
        self.R_w = None
        self.reset()

    def attach_driveline(self, i_final, R_w):
        self.i_final = i_final
        self.R_w = R_w

    def reset(self, delta_f=0.0, delta_r=0.0, T_f=0.0, T_r=0.0):
        self.delta_f, self.delta_r, self.T_f, self.T_r = delta_f, delta_r, T_f, T_r

    def _slew(self, current, target, rate, dt):
        step = rate * dt
        return current + float(np.clip(target - current, -step, step))

    def _torque_cap(self, T_cmd, v_x):
        cap = self.T_max_drive if T_cmd >= 0 else self.T_max_brake
        if np.isfinite(self.P_max) and self.i_final and self.R_w and v_x > 0.1:
            omega_m = self.i_final * v_x / self.R_w
            if omega_m > 1e-3:
                cap = min(cap, self.P_max / omega_m)
        return float(np.clip(T_cmd, -cap, cap)) if np.isfinite(cap) else float(T_cmd)

    def _motor(self, current, T_cmd, v_x, dt):
        target = self._torque_cap(T_cmd, v_x)
        if self.torque_tau > 0:
            target = current + (dt / (self.torque_tau + dt)) * (target - current)
        dT = float(np.clip(target - current, -self.torque_rate_max * dt, self.torque_rate_max * dt))
        return current + dT

    def step(self, cmd, v_x, dt):
        df_cmd, dr_cmd, Tf_cmd, Tr_cmd = cmd
        df_cmd = float(np.clip(df_cmd, -self.ang_max_f, self.ang_max_f))
        dr_cmd = float(np.clip(dr_cmd, -self.ang_max_r, self.ang_max_r))
        self.delta_f = self._slew(self.delta_f, df_cmd, self.steer_rate_rw, dt)
        self.delta_r = self._slew(self.delta_r, dr_cmd, self.steer_rate_rw, dt)
        self.T_f = self._motor(self.T_f, Tf_cmd, v_x, dt)
        self.T_r = self._motor(self.T_r, Tr_cmd, v_x, dt)
        return np.array([self.delta_f, self.delta_r, self.T_f, self.T_r])


# ======================================================================
# 构建
# ======================================================================

def build_vehicle_6state_dyn(cfg: dict) -> FourWSVehicle6StateDyn:
    v_cfg = cfg.get("vehicle", {})
    d = cfg.get("vehicle_6state_dyn", {})
    vj = cfg["vehicle_json"]
    geom = vj["geometry"]
    mi = vj["mass_inertia"]
    env = vj.get("environment", {})

    wheelbase = v_cfg.get("wheelbase") or geom.get("wheelbase")
    mass = d.get("mass") or mi["mass_awk"]
    I_z = d.get("I_z") or mi.get("yaw_inertia_izz")   # None -> 模型内按 m*a*b 估算
    return FourWSVehicle6StateDyn(
        wheelbase=float(wheelbase),
        a_dist=geom["a_dist"],
        b_dist=geom["b_dist"],
        mass=float(mass),
        I_z=(float(I_z) if I_z else None),
        C_alpha_f=float(d.get("C_alpha_f", 180000.0)),
        C_alpha_r=float(d.get("C_alpha_r", 180000.0)),
        mu0=float(d.get("mu0", 0.85)),
        c_roll=float(d.get("c_roll", 0.015)),
        cd_a=float(d.get("cd_a", 6.0)),
        rho=float(d.get("rho", 1.225)),
        grade_percent=float(d.get("grade_percent", 0.0)),
        wheel_radius=float(geom.get("wheel_radius", 0.402)),
        final_drive_ratio=float(vj["drivetrain"]["final_drive_ratio"]),
        eta_drive=float(d.get("eta_drive", 0.92)),
        h_cg=float(d.get("h_cg") or geom.get("cg_height") or 1.1),
        motor_inertia=float(mi.get("motor_inertia", 0.0) or 0.0),
        n_drive_motors=int(d.get("n_drive_motors", 2)),
        max_slope_percent=float(env.get("max_slope_percent", 12.0)),
    )


def build_actuators(cfg: dict) -> VehicleActuators:
    vj = cfg["vehicle_json"]
    st = vj.get("steering", {})
    geom = vj["geometry"]
    dr = vj["drivetrain"]
    act = VehicleActuators(
        steer_ratio=float(st.get("steer_ratio", 46.7)),
        steer_rate_limit_input_deg_s=float(st.get("steer_rate_limit_input_deg_s", 520.0)),
        steer_angle_max_front_deg=float(st.get("steer_angle_max_front_deg") or 15.0),
        steer_angle_max_rear_deg=float(st.get("steer_angle_max_rear_deg") or 15.0),
    )
    act.attach_driveline(float(dr["final_drive_ratio"]), float(geom.get("wheel_radius", 0.402)))
    return act
