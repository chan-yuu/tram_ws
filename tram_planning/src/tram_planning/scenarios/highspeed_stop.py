"""高速站点停靠: 80 km/h → 停 → 80 km/h

场景描述:
  以恒定加速度从静止加速到 V → 巡航 → 减速进站 → 停靠 →
  恒加速出站 → 巡航 → 减速终点
  关键: 进/出站加减速段用恒加速度 (sqrt 速度曲线), 长度由 V²/(2·a) 决定,
  避免老版 make_straight 给出的 v_end²/L 在大 V 短距离上爆炸式加速度.
"""
from .base_scenario import (BaseScenario, WayPoint, make_straight,
                             make_const_accel_straight, mark_stop_point, concat)
from typing import List


V = 80.0 / 3.6   # 22.222 m/s
A_NOM = 2.5      # m/s², leaves margin under controller max_accel (3.5)
A_BRAKE = 3.0    # m/s², comfortable service brake


class HighspeedStop(BaseScenario):
    name = "highspeed_stop"
    description = "80 km/h station stop with physically realizable accel/decel ramps"

    def _raw_segments(self, ds: float) -> List[WayPoint]:
        x, y, yaw = 0.0, 0.0, 0.0
        # Accelerate 0 → V at constant A_NOM (~99 m for V=22.22)
        s0 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s0[-1].s
        # Cruise at V
        s1 = make_straight(x, y, yaw, 60.0, V, V, ds)
        x += 60.0
        # Brake V → 0 at constant A_BRAKE (~82 m)
        s2 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s2 = mark_stop_point(s2, dwell_time=5.0)
        x += s2[-1].s
        # Restart 0 → V
        s3 = make_const_accel_straight(x, y, yaw, 0.0, V, A_NOM, ds)
        x += s3[-1].s
        # Cruise
        s4 = make_straight(x, y, yaw, 60.0, V, V, ds)
        x += 60.0
        # Final brake to stop
        s5 = make_const_accel_straight(x, y, yaw, V, 0.0, A_BRAKE, ds)
        s5 = mark_stop_point(s5, dwell_time=0.0)
        return concat(s0, s1, s2, s3, s4, s5)
