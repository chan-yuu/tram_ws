"""Longitudinal PI speed controller with anti-windup."""
import numpy as np


class PISpeedController:
    def __init__(self, kp: float = 1.5, ki: float = 0.3,
                 max_accel: float = 2.0, max_decel: float = 3.5,
                 windup_limit: float = 5.0):
        self.kp = kp
        self.ki = ki
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.windup_limit = windup_limit
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def compute(self, v_ref: float, v_actual: float, dt: float) -> float:
        """Return longitudinal acceleration command (m/s²)."""
        error = v_ref - v_actual
        self._integral += error * dt
        a_cmd = self.kp * error + self.ki * self._integral

        # Back-calculation anti-windup: only correct when output saturates
        a_clamped = float(np.clip(a_cmd, -self.max_decel, self.max_accel))
        if abs(a_cmd) > abs(a_clamped):
            excess = (a_cmd - a_clamped) / (self.ki + 1e-9)
            self._integral -= excess

        return a_clamped
