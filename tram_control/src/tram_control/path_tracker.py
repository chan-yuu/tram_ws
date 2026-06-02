"""Path tracking utilities: nearest-point search and error computation."""
import numpy as np
from typing import List, Tuple, Optional
from tram_msgs.msg import PathPoint


def find_nearest(points: List[PathPoint], x: float, y: float,
                 start_idx: int = 0, search_window: int = 150) -> Tuple[int, float]:
    """Return (index, distance) of the nearest path point.

    Searches in a window ahead of start_idx to avoid backward jumps.
    """
    n = len(points)
    if n == 0:
        return 0, 0.0
    end = min(start_idx + search_window, n)
    # Also allow a small lookback to handle deceleration overlaps
    begin = max(0, start_idx - 5)

    best_idx = begin
    best_d2 = float('inf')
    for i in range(begin, end):
        d2 = (points[i].x - x) ** 2 + (points[i].y - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    return best_idx, float(np.sqrt(best_d2))


def compute_errors(points: List[PathPoint], idx: int,
                   x: float, y: float, psi: float) -> Tuple[float, float]:
    """Return (lateral_error, heading_error) at the given path index.

    lateral_error > 0  →  vehicle is to the LEFT of path (needs right steer).
    heading_error > 0  →  vehicle heading is left of path tangent.
    """
    ref = points[idx]
    dx = x - ref.x
    dy = y - ref.y
    # Signed cross-track error in the path frame
    # e_y = -sin(yaw)*dx + cos(yaw)*dy
    e_y = -np.sin(ref.yaw) * dx + np.cos(ref.yaw) * dy

    e_psi = _wrap(psi - ref.yaw)
    return e_y, e_psi


def estimate_progress_s(points: List[PathPoint], idx: int,
                        x: float, y: float) -> float:
    """Project the current position onto nearby path segments and return arc length."""
    if not points:
        return 0.0
    candidates = []
    n = len(points)
    for i0, i1 in ((idx - 1, idx), (idx, idx + 1)):
        if i0 < 0 or i1 >= n:
            continue
        p0 = points[i0]
        p1 = points[i1]
        vx = p1.x - p0.x
        vy = p1.y - p0.y
        seg_len2 = vx * vx + vy * vy
        if seg_len2 < 1e-9:
            continue
        t = ((x - p0.x) * vx + (y - p0.y) * vy) / seg_len2
        t = float(np.clip(t, 0.0, 1.0))
        px = p0.x + t * vx
        py = p0.y + t * vy
        d2 = (x - px) ** 2 + (y - py) ** 2
        s = p0.s + t * (p1.s - p0.s)
        candidates.append((d2, s))
    if candidates:
        return min(candidates, key=lambda item: item[0])[1]
    return float(points[min(max(idx, 0), n - 1)].s)


def get_reference_segment(points: List[PathPoint], idx: int,
                           horizon: int) -> List[PathPoint]:
    """Return up to `horizon` path points starting at idx (clamps at end)."""
    end = min(idx + horizon, len(points))
    return points[idx:end]


def interpolate_ref(points: List[PathPoint], idx: int,
                    v_x: float, dt: float) -> PathPoint:
    """Lookahead reference: step forward by v_x*dt arc length from idx."""
    if not points:
        return PathPoint()
    target_s = points[idx].s + max(v_x * dt, 0.5)
    for i in range(idx, len(points)):
        if points[i].s >= target_s:
            return points[i]
    return points[-1]


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
