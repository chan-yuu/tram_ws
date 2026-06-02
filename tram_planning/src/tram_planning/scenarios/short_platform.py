"""短站台停靠: 40 km/h → 停 → 40 km/h

恒加速度起步, 巡航, 减速进站, 紧凑站台, 出站再加速, 巡航, 终点停车.
"""
from .base_scenario import (BaseScenario, WayPoint, make_straight,
                             make_const_accel_straight, mark_stop_point, concat)
from typing import List


V = 40.0 / 3.6   # 11.111 m/s
A_NOM = 2.0      # m/s², gentler since target speed is lower
A_BRAKE = 2.0


class ShortPlatform(BaseScenario):
    name = "short_platform"
    description = "40 km/h short platform stop with physically realizable accel/decel ramps"

    def _raw_segments(self, ds: float) -> List[WayPoint]:
        x, y, yaw = 0.0, 0.0, 0.0
        # 0 → V (~31 m)
        s0 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s0[-1].s
        s1 = make_straight(x, y, yaw, 30.0, V, V, ds)
        x += 30.0
        s2 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s2 = mark_stop_point(s2, dwell_time=5.0)
        x += s2[-1].s
        s3 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s3[-1].s
        s4 = make_straight(x, y, yaw, 20.0, V, V, ds)
        x += 20.0
        s5 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s5 = mark_stop_point(s5, dwell_time=0.0)
        return concat(s0, s1, s2, s3, s4, s5)
