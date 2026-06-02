"""标准站点停靠: 60 km/h → 停 → 60 km/h

恒加速度起步, 巡航, 恒减速进站, 站台停靠, 出站再恒加速到 V, 巡航, 终点停车.
关键: 加减速段长度由 V²/(2·a) 决定, 让控制器有可能实际跟踪到目标速度.
"""
from .base_scenario import (BaseScenario, WayPoint, make_straight,
                             make_const_accel_straight, mark_stop_point, concat)
from typing import List


V = 60.0 / 3.6   # 16.667 m/s
A_NOM = 2.5      # m/s² accel
A_BRAKE = 2.5    # m/s² decel


class StandardStop(BaseScenario):
    name = "standard_stop"
    description = "60 km/h station stop with physically realizable accel/decel ramps"

    def _raw_segments(self, ds: float) -> List[WayPoint]:
        x, y, yaw = 0.0, 0.0, 0.0
        # 0 → V (~56 m)
        s0 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s0[-1].s
        s1 = make_straight(x, y, yaw, 90.0, V, V, ds)
        x += 90.0
        # V → 0 (~56 m)
        s2 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s2 = mark_stop_point(s2, dwell_time=5.0)
        x += s2[-1].s
        # Restart 0 → V
        s3 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s3[-1].s
        s4 = make_straight(x, y, yaw, 90.0, V, V, ds)
        x += 90.0
        s5 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s5 = mark_stop_point(s5, dwell_time=0.0)
        return concat(s0, s1, s2, s3, s4, s5)
