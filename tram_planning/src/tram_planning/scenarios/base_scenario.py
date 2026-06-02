"""Abstract base class and geometry helpers for path scenarios."""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List
import numpy as np


@dataclass
class WayPoint:
    x: float
    y: float
    yaw: float       # path tangent heading (rad)
    v: float         # target speed (m/s)
    s: float = 0.0   # cumulative arc length (m)
    kappa: float = 0.0  # path curvature 1/R (1/m)
    stop_point: bool = False
    dwell_time: float = 0.0


class BaseScenario(ABC):
    name: str = "base"
    description: str = ""

    @abstractmethod
    def _raw_segments(self, ds: float) -> List[WayPoint]:
        """Return densely-sampled waypoints (ds ≈ 0.5m) for the scenario."""
        ...

    def generate(self, ds: float = 0.5) -> List[WayPoint]:
        """Return path with cumulative s and curvature κ computed."""
        pts = self._raw_segments(ds)
        if len(pts) < 2:
            return pts
        # recompute cumulative arc length
        s = 0.0
        pts[0].s = 0.0
        for i in range(1, len(pts)):
            ds_i = np.hypot(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y)
            s += ds_i
            pts[i].s = s
        return _compute_kappa(pts)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def make_straight(x0: float, y0: float, yaw: float,
                  length: float, v_start: float, v_end: float,
                  ds: float = 0.5) -> List[WayPoint]:
    """Uniformly-sampled straight segment with linear speed ramp."""
    n = max(2, int(round(length / ds)))
    pts: List[WayPoint] = []
    for i in range(n + 1):
        t = i / n
        pts.append(WayPoint(
            x=x0 + t * length * np.cos(yaw),
            y=y0 + t * length * np.sin(yaw),
            yaw=yaw,
            v=max(v_start + t * (v_end - v_start), 0.0),
            s=t * length,
        ))
    return pts


def make_const_accel_straight(x0: float, y0: float, yaw: float,
                              v_start: float, v_end: float, accel: float,
                              ds: float = 0.5) -> List[WayPoint]:
    """Straight segment with **constant longitudinal acceleration**.

    Speed profile follows v(s) = sqrt(v_start² ± 2·a·s), which gives uniform
    a_lon throughout the segment (unlike `make_straight`'s linear v(s) ramp,
    whose peak acceleration v_end²/L blows up for short ramps to high speeds).
    Use this for station start/restart accel and approach-decel zones where
    the controller would otherwise have to chase an unrealizable v_ref step.
    """
    accel = abs(accel)
    sign = 1.0 if v_end >= v_start else -1.0
    L = abs(v_end * v_end - v_start * v_start) / (2.0 * max(accel, 1e-6))
    if L < 1e-6:
        return [WayPoint(x=x0, y=y0, yaw=yaw, v=v_start, s=0.0)]
    n = max(2, int(round(L / ds)))
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    pts: List[WayPoint] = []
    for i in range(n + 1):
        s = (i / n) * L
        v_sq = v_start * v_start + sign * 2.0 * accel * s
        v = float(np.sqrt(max(v_sq, 0.0)))
        pts.append(WayPoint(
            x=x0 + s * cos_y,
            y=y0 + s * sin_y,
            yaw=yaw,
            v=v,
            s=s,
        ))
    return pts


def mark_stop_point(points: List[WayPoint], dwell_time: float = 5.0) -> List[WayPoint]:
    """Mark the final point of a segment as an operational stop."""
    if points:
        points[-1].stop_point = True
        points[-1].dwell_time = dwell_time
        points[-1].v = 0.0
    return points


def make_arc(x_center: float, y_center: float, radius: float,
             yaw_start: float, sweep_rad: float, v: float,
             ds: float = 0.5) -> List[WayPoint]:
    """Circular arc sampled at ~ds intervals.

    Args:
        yaw_start:  path tangent heading at the START of the arc (rad).
        sweep_rad:  total heading change; positive = CCW (left turn),
                    negative = CW (right turn).
    Point position given heading θ:
        CCW (center left):  P = C + R*(cos(θ-π/2), sin(θ-π/2))
        CW  (center right): P = C + R*(cos(θ+π/2), sin(θ+π/2))
    """
    arc_len = abs(radius * sweep_rad)
    n = max(2, int(round(arc_len / ds)))
    sign = 1 if sweep_rad >= 0 else -1
    pts: List[WayPoint] = []
    for i in range(n + 1):
        t = i / n
        theta = yaw_start + t * sweep_rad
        offset = theta - sign * np.pi / 2
        pts.append(WayPoint(
            x=x_center + radius * np.cos(offset),
            y=y_center + radius * np.sin(offset),
            yaw=theta,
            v=v,
            s=t * arc_len,
        ))
    return pts


def make_cubic_lane_change(x0: float, y0: float, yaw0: float,
                           exit_yaw: float, length: float,
                           v: float, ds: float = 0.5) -> List[WayPoint]:
    """Smooth lane-change using a cubic-polynomial lateral profile.

    Boundary conditions:
        x(0)=x0, y(0)=y0, dy/dx(0)=tan(yaw0)
        dy/dx(L)=tan(exit_yaw), with cubic fit in local frame.
    The lateral offset at x=L is L*tan(exit_yaw) (8-degree fork → ≈8.43m/60m).
    """
    tan1 = np.tan(exit_yaw - yaw0)
    L = length

    # Cubic y(x) = ax³ + bx² satisfying y(0)=y'(0)=0, y'(L)=tan1, y(L)=L*tan1
    # Solution: a = -tan1/L², b = 2*tan1/L
    a = -tan1 / (L ** 2)
    b = 2 * tan1 / L

    n = max(2, int(round(L / ds)))
    pts: List[WayPoint] = []
    cos0, sin0 = np.cos(yaw0), np.sin(yaw0)
    for i in range(n + 1):
        t = i / n
        xl = t * L
        yl = a * xl**3 + b * xl**2
        # slope in local frame
        dydx = 3 * a * xl**2 + 2 * b * xl
        local_yaw = np.arctan(dydx)
        # rotate to global frame
        x = x0 + xl * cos0 - yl * sin0
        y = y0 + xl * sin0 + yl * cos0
        path_yaw = yaw0 + local_yaw
        pts.append(WayPoint(x=x, y=y, yaw=path_yaw, v=v, s=t * L))
    return pts


def _interp_angle(a0: float, a1: float, t: float) -> float:
    diff = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
    return a0 + t * diff


def _compute_kappa(pts: List[WayPoint]) -> List[WayPoint]:
    n = len(pts)
    for i in range(n):
        if i == 0:
            dyaw = _wrap(pts[1].yaw - pts[0].yaw)
            ds_ = max(pts[1].s - pts[0].s, 1e-6)
        elif i == n - 1:
            dyaw = _wrap(pts[-1].yaw - pts[-2].yaw)
            ds_ = max(pts[-1].s - pts[-2].s, 1e-6)
        else:
            dyaw = _wrap(pts[i+1].yaw - pts[i-1].yaw)
            ds_ = max(pts[i+1].s - pts[i-1].s, 1e-6)
        pts[i].kappa = dyaw / ds_
    return pts


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def concat(*segment_lists) -> List[WayPoint]:
    """Concatenate segments, offsetting s of each by the previous total."""
    result: List[WayPoint] = []
    s_offset = 0.0
    for seg in segment_lists:
        if not seg:
            continue
        if result:
            s_offset = result[-1].s
            start = 1  # skip duplicate join point
        else:
            start = 0
        for p in seg[start:]:
            result.append(WayPoint(
                x=p.x, y=p.y, yaw=p.yaw, v=p.v,
                s=s_offset + p.s, stop_point=p.stop_point,
                dwell_time=p.dwell_time,
            ))
    return result
