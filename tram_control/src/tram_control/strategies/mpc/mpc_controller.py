"""Linear MPC for lateral path following (4WS tram).

State:   z = [e_y, e_psi, v_y, r]          (4-dim lateral error state)
Control: u = [delta_f, delta_r]             (road-wheel angles, rad)
Horizon: N steps of dt_mpc seconds.

Formulation (condensed QP):
  Z = Phi * z0 + Gamma * U
  min_U  U.T H U + f.T U
     s.t. lb <= U <= ub
  H = Gamma.T Q_bar Gamma + R_bar
  f = 2 Gamma.T Q_bar Phi z0

Longitudinal speed is controlled separately by PI; v_x is a scheduling param.
"""
import numpy as np


class MPCLateralController:
    def __init__(self,
                 Cf: float = 180000.0,
                 Cr: float = 180000.0,
                 mass: float = 11000.0,
                 Iz: float = 158910.0,
                 a_dist: float = 3.638,
                 b_dist: float = 3.962,
                 L: float = 7.600,
                 N: int = 15,
                 dt_mpc: float = 0.05,
                 Q=None,
                 R=None,
                 delta_max: float = 0.2618,
                 delta_rate_max: float = 0.1,   # rad/step
                 vx_min: float = 0.5):
        self.Cf = Cf; self.Cr = Cr
        self.m = mass; self.Iz = Iz
        self.a = a_dist; self.b = b_dist; self.L = L
        self.N = N
        self.dt = dt_mpc
        self.delta_max = delta_max
        self.delta_rate_max = delta_rate_max
        self.vx_min = vx_min

        self.Q = self._coerce_matrix(Q, np.diag([8.0, 3.0, 0.1, 0.1]))
        # R may be a 2-vector (diagonal) or a 2x2 matrix (with off-diagonals
        # to penalise same-phase steering, i.e. (delta_f + delta_r)^2).
        self.R = self._coerce_matrix(R, np.diag([1.0, 1.0]))

        self._prev_u = np.zeros(2)
        self._last_vx = None
        self._Phi = None
        self._Gamma = None
        self._H = None
        self._Q_bar = np.kron(np.eye(N), self.Q)
        self._R_bar = np.kron(np.eye(N), self.R)

    @staticmethod
    def _coerce_matrix(value, default):
        """Accept a 1-D (diagonal) or 2-D (full) cost matrix specification."""
        if value is None:
            return default
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 1:
            return np.diag(arr)
        return arr

    # ------------------------------------------------------------------
    def _build_linear_model(self, vx: float):
        vx = max(vx, self.vx_min)
        Cf, Cr = self.Cf, self.Cr
        m, Iz = self.m, self.Iz
        a, b = self.a, self.b
        dt = self.dt

        A = np.array([
            [0,  vx,  1,  0],
            [0,   0,  0,  1],
            [0,   0, -(Cf+Cr)/(m*vx), -(vx + (Cf*a - Cr*b)/(m*vx))],
            [0,   0,  (Cr*b-Cf*a)/(Iz*vx), -(Cf*a**2+Cr*b**2)/(Iz*vx)],
        ])
        B = np.array([
            [0,        0    ],
            [0,        0    ],
            [Cf/m,     Cr/m ],
            [Cf*a/Iz, -Cr*b/Iz],
        ])
        # Tustin (bilinear) discretization
        I = np.eye(4)
        inv_term = np.linalg.inv(I - 0.5 * dt * A)
        Ad = inv_term @ (I + 0.5 * dt * A)
        Bd = inv_term @ B * dt
        return Ad, Bd

    def _build_prediction_matrices(self, Ad, Bd):
        n, m = 4, 2
        N = self.N
        Phi = np.zeros((n * N, n))
        Gamma = np.zeros((n * N, m * N))

        Ak = Ad.copy()
        for k in range(N):
            Phi[k*n:(k+1)*n, :] = Ak
            Ak = Ad @ Ak

        for j in range(N):
            AB = Bd.copy()
            for k in range(j, N):
                Gamma[k*n:(k+1)*n, j*m:(j+1)*m] = AB
                AB = Ad @ AB

        return Phi, Gamma

    def _update_matrices(self, vx: float):
        Ad, Bd = self._build_linear_model(vx)
        self._Phi, self._Gamma = self._build_prediction_matrices(Ad, Bd)
        G = self._Gamma
        H = G.T @ self._Q_bar @ G + self._R_bar
        # Regularize for numerical stability
        H += 1e-6 * np.eye(H.shape[0])
        self._H = H
        self._last_vx = vx

    # ------------------------------------------------------------------
    def compute(self, e_y: float, e_psi: float,
                v_y: float, r: float, v_x: float,
                kappa: float = 0.0) -> np.ndarray:
        """Return [delta_f, delta_r] in radians."""
        vx = max(v_x, self.vx_min)
        if self._last_vx is None or abs(v_x - self._last_vx) > 0.5:
            self._update_matrices(v_x)

        # On a curved path the reference yaw rate is κ·vx, not zero.
        # Using raw r would make the MPC interpret steady circular motion as
        # a large spinning error and overreact.
        r_err = r - kappa * vx
        z0 = np.array([e_y, e_psi, v_y, r_err])
        G = self._Gamma
        f = 2.0 * G.T @ self._Q_bar @ self._Phi @ z0

        # Unconstrained analytical solution
        try:
            U_opt = -0.5 * np.linalg.solve(self._H, f)
        except np.linalg.LinAlgError:
            U_opt = np.zeros(2 * self.N)

        # Input constraints (box): reshape to (N, 2), clamp
        U_opt = U_opt.reshape(self.N, 2)
        U_opt = np.clip(U_opt, -self.delta_max, self.delta_max)

        # Curvature feedforward for first step
        ff = np.array([np.arctan(self.a * kappa), -np.arctan(self.b * kappa)])

        # Rate-limit the TOTAL command (feedback + feedforward).
        # Previously _prev_u tracked the total but du was applied only to the
        # feedback portion, so the feedforward re-absorbed each rate-limited
        # decrement and the command was permanently pinned at saturation.
        desired_total = np.clip(U_opt[0] + ff, -self.delta_max, self.delta_max)
        du = desired_total - self._prev_u
        du = np.clip(du, -self.delta_rate_max, self.delta_rate_max)
        actual_u = self._prev_u + du

        self._prev_u = actual_u.copy()
        return actual_u

    def reset(self):
        self._prev_u = np.zeros(2)
