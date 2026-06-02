"""完整道岔换道: 8° 分岔

场景描述:
  150m 主轨道巡航 → 60m 道岔过渡(8°分岔角) → 150m 侧线巡航
  验证横向跟踪与前后轮协同
"""
import numpy as np
from .base_scenario import (BaseScenario, WayPoint,
                             make_straight, make_cubic_lane_change,
                             mark_stop_point, concat)
from typing import List


FORK_DEG = 8.0
V = 30.0 / 3.6   # 8.333 m/s
TRANSITION_LEN = 60.0


class SwitchLane(BaseScenario):
    name = "switch_lane"
    description = "8-degree fork lane change: 150m main + 60m cubic transition + 150m side track"

    def _raw_segments(self, ds: float) -> List[WayPoint]:
        yaw_main = 0.0
        yaw_side = np.deg2rad(FORK_DEG)

        x0, y0 = 0.0, 0.0
        s1 = make_straight(x0, y0, yaw_main, 150.0, V, V, ds)

        # Start of transition
        x1 = x0 + 150.0 * np.cos(yaw_main)
        y1 = y0 + 150.0 * np.sin(yaw_main)
        s2 = make_cubic_lane_change(x1, y1, yaw_main, yaw_side,
                                    TRANSITION_LEN, V, ds)

        # Start of side track
        x2 = s2[-1].x
        y2 = s2[-1].y
        s3 = make_straight(x2, y2, yaw_side, 90.0, V, V, ds)
        x3 = s3[-1].x
        y3 = s3[-1].y
        s4 = mark_stop_point(make_straight(x3, y3, yaw_side, 60.0, V, 0.0, ds), dwell_time=0.0)

        return concat(s1, s2, s3, s4)
