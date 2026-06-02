"""Lateral LQR controller for 4WS tram.

State vector: x = [e_y, e_psi, v_y, r]
  e_y   — lateral position error (m), positive = vehicle left of path
  e_psi — heading error (rad), positive = vehicle heading left of path
  v_y   — lateral body velocity (m/s)
  r     — yaw rate (rad/s)

Control: u = [delta_f, delta_r] — front/rear road-wheel angles (rad)

The gain matrix K is re-computed every call when v_x changes by > 0.5 m/s
(scheduled LQR / gain scheduling).
"""
import numpy as np
from scipy.linalg import solve_discrete_are


class LQRLateralController:
    def __init__(self,
                 Cf: float = 180000.0,   # N/rad front cornering stiffness
                 Cr: float = 180000.0,   # N/rad rear cornering stiffness
                 mass: float = 11000.0,  # kg
                 Iz: float = 158910.0,   # kg·m²  (m*a*b fallback)
                 a_dist: float = 3.638,  # m front axle to CG
                 b_dist: float = 3.962,  # m rear axle to CG
                 L: float = 7.600,       # m wheelbase
                 Q=None,                 # 4×4 state cost
                 R=None,                 # 2×2 input cost
                 dt: float = 0.02,
                 delta_max: float = 0.2618,   # ≈ 15° max road-wheel angle
                 vx_min: float = 0.5):
        self.Cf = Cf; self.Cr = Cr
        self.m = mass; self.Iz = Iz
        self.a = a_dist; self.b = b_dist; self.L = L
        self.dt = dt
        self.delta_max = delta_max
        self.vx_min = vx_min

        self.Q = np.diag(Q) if Q is not None else np.diag([5.0, 2.0, 0.1, 0.1])
        self.R = np.diag(R) if R is not None else np.diag([0.5, 0.5])

        self._K = None
        self._last_vx = None

    def _build_matrices(self, vx: float):
        """Return continuous system matrices (A, B)."""
        vx = max(vx, self.vx_min)
        Cf, Cr = self.Cf, self.Cr
        m, Iz = self.m, self.Iz
        a, b = self.a, self.b

        A = np.array([
            [0,  vx,  1,  0],
            [0,   0,  0,  1],
            [0,   0, -(Cf+Cr)/(m*vx), -(vx + (Cf*a - Cr*b)/(m*vx))],
            [0,   0,  (Cr*b-Cf*a)/(Iz*vx), -(Cf*a**2+Cr*b**2)/(Iz*vx)],
        ])
        B = np.array([
            [0,          0      ],
            [0,          0      ],
            [Cf/m,       Cr/m   ],
            [Cf*a/Iz,   -Cr*b/Iz],
        ])
        return A, B

    def _update_gain(self, vx: float):
        A, B = self._build_matrices(vx)
        # Discrete-time Bilinear (Tustin) approximation
        I = np.eye(4)
        try:
            inv_term = np.linalg.inv(I - 0.5 * self.dt * A)
            Ad = inv_term @ (I + 0.5 * self.dt * A)
            Bd = inv_term @ B * self.dt
            P = solve_discrete_are(Ad, Bd, self.Q, self.R)
            K = np.linalg.solve(self.R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        except Exception:
            K = np.zeros((2, 4))
        self._K = K
        self._last_vx = vx

    # ------------------------------------------------------------------
    def compute(self, e_y: float, e_psi: float,
                v_y: float, r: float, v_x: float,
                kappa: float = 0.0) -> np.ndarray:
        """Return [delta_f, delta_r] in radians.

        Args:
            kappa: path curvature at current reference point (1/m) for
                   Ackermann feedforward.
        """
        if self._last_vx is None or abs(v_x - self._last_vx) > 0.5:
            self._update_gain(v_x)

        x_err = np.array([e_y, e_psi, v_y, r])
        u_fb = -self._K @ x_err

        # 4WS curvature feedforward: exact Ackermann steering for neutral sideslip
        delta_ff_f = np.arctan(self.a * kappa)
        delta_ff_r = -np.arctan(self.b * kappa)

        u = u_fb + np.array([delta_ff_f, delta_ff_r])
        # Clamp to physical limits
        u = np.clip(u, -self.delta_max, self.delta_max)
        return u
