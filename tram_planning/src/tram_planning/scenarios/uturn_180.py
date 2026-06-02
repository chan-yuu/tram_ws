"""180° 弯道掉头: R=15m, curved section v=3 m/s (低速 → 出弯振荡小)

场景描述:
  50m 直线巡航 → 25m 弯前减速 → 5m 弯前稳态 →
  R=15m 半圆(180°) → 10m 出弯稳态 → 20m 出弯加速 → 20m 巡航 → 40m 减速停车
  关键改动: 出弯加 POST_TURN_SETTLE 段保持低速, 让控制器先消掉
  曲率阶跃带来的偏航角速度残量, 再开始加速; 入弯同理保留小段
  稳态段避免减速末段直接进入曲率阶跃.
"""
import numpy as np
from .base_scenario import (BaseScenario, WayPoint,
                             make_straight, make_arc, mark_stop_point, concat)
from typing import List


V_APPROACH = 8.0     # m/s, straight approach
V_TURN = 3.0         # m/s, R=15m → ay≈0.6 m/s², yaw_rate≈0.2 rad/s
V_EXIT = 6.0         # m/s, return straight cruise before final stop
RADIUS = 15.0
STRAIGHT = 80.0
PRE_TURN_SETTLE = 5.0   # m at V_TURN before curve entry
POST_TURN_SETTLE = 10.0  # m at V_TURN after curve exit (key to damp oscillation)


class UTurn180(BaseScenario):
    name = "uturn_180"
    description = "180-deg U-turn R=15m at v=3 m/s with pre/post settle straights"

    def _raw_segments(self, ds: float) -> List[WayPoint]:
        # --- Segment 1: approach straight (heading east, yaw=0) ---
        s1 = make_straight(0.0, 0.0, 0.0, 50.0, V_APPROACH, V_APPROACH, ds)
        # Decel directly to V_TURN, finishing PRE_TURN_SETTLE meters before the curve
        decel_len = max(STRAIGHT - 50.0 - PRE_TURN_SETTLE, 1.0)
        s2 = make_straight(50.0, 0.0, 0.0, decel_len, V_APPROACH, V_TURN, ds)
        # --- Pre-turn settle: hold V_TURN steady right up to the arc tangent point ---
        x2 = s2[-1].x; y2 = s2[-1].y
        s2b = make_straight(x2, y2, 0.0, PRE_TURN_SETTLE, V_TURN, V_TURN, ds)

        # --- Segment 3: CCW semicircle ---
        # Arc entry at (STRAIGHT, 0) heading east; centre to the LEFT → (STRAIGHT, RADIUS)
        s3 = make_arc(STRAIGHT, RADIUS, RADIUS,
                      yaw_start=0.0, sweep_rad=np.pi, v=V_TURN, ds=ds)

        # --- Post-turn settle: hold V_TURN, give the controller time to bleed off
        # the yaw rate / sideslip that built up on the arc before the path begins
        # to accelerate again. Without this, the curvature-step at exit (κ jumps
        # from 0.0667 → 0) and the simultaneous longitudinal accel create a heading
        # oscillation ("snaking") that takes several seconds to damp.
        x3 = s3[-1].x; y3 = s3[-1].y
        s4a = make_straight(x3, y3, np.pi, POST_TURN_SETTLE, V_TURN, V_TURN, ds)

        # --- Accel back up to cruise speed, then cruise, then decel to final stop ---
        x4 = s4a[-1].x; y4 = s4a[-1].y
        s4b = make_straight(x4, y4, np.pi, 20.0, V_TURN, V_EXIT, ds)
        x5 = s4b[-1].x; y5 = s4b[-1].y
        s5 = make_straight(x5, y5, np.pi, 20.0, V_EXIT, V_EXIT, ds)
        x6 = s5[-1].x; y6 = s5[-1].y
        s6 = mark_stop_point(make_straight(x6, y6, np.pi, 40.0, V_EXIT, 0.0, ds), dwell_time=0.0)

        return concat(s1, s2, s2b, s3, s4a, s4b, s5, s6)
