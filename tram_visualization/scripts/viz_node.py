#!/usr/bin/env python3
"""Visualization node — publishes MarkerArrays for RViz.

Publishes:
  /viz/vehicle_marker    visualization_msgs/MarkerArray  (vehicle box + axes)
                         includes a text status board following the tram
  /viz/path_markers      visualization_msgs/MarkerArray  (path points, colored by speed)
  /viz/key_points        visualization_msgs/MarkerArray  (start/stop/end/curve key points)
  /viz/control_markers   visualization_msgs/MarkerArray  (nearest point, error arrow)
  /viz/lookahead_marker  visualization_msgs/Marker       (lookahead target sphere)
"""
import numpy as np
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Float64, Header, String
from tram_msgs.msg import VehicleState, TramPath


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _point(x, y, z=0.0):
    return Point(x=x, y=y, z=z)


def _fix_quat(obj):
    """Set uninitialized quaternion to identity to silence RViz warnings."""
    if hasattr(obj, 'markers'):
        for m in obj.markers:
            if m.pose.orientation.w == 0.0:
                m.pose.orientation.w = 1.0
    elif hasattr(obj, 'pose'):
        if obj.pose.orientation.w == 0.0:
            obj.pose.orientation.w = 1.0
    return obj


class VizNode:
    # Mesh dimensions (car.obj bounding box)
    BODY_L = 11.25
    BODY_W = 2.47
    BODY_H = 3.35

    def __init__(self):
        rospy.init_node('viz_node', anonymous=False)
        self.freq = rospy.get_param('~freq', 20.0)

        # Text display params (tune via rosparam if needed)
        self._text_speed_z    = rospy.get_param('~text_speed_z',    4.25)
        self._text_speed_size = rospy.get_param('~text_speed_size', 1.5)
        self._text_status_z    = rospy.get_param('~text_status_z',    5.5)
        self._text_status_size = rospy.get_param('~text_status_size', 1.0)
        self._text_lat_offset  = rospy.get_param('~text_lat_offset',  6.8)
        self._text_fwd_offset  = rospy.get_param('~text_fwd_offset',  2.0)

        self.state: VehicleState = None
        self.path_points = []
        self.nearest_idx = 0
        self.scenario_name = "unknown"
        self.speed_ref = float("nan")
        self.stop_state = "none"
        self.stop_distance = float("nan")
        self.dwell_elapsed = float("nan")
        self.dwell_remaining = float("nan")

        rospy.Subscriber('/sim/vehicle_state', VehicleState, self._cb_state, queue_size=1)
        rospy.Subscriber('/planning/path',     TramPath,     self._cb_path,  queue_size=1)
        rospy.Subscriber('/planning/scenario', String,       self._cb_scenario, queue_size=1)
        rospy.Subscriber('/debug/progress', Float64, self._cb_progress, queue_size=1)
        rospy.Subscriber('/debug/speed_ref', Float64, self._cb_speed_ref, queue_size=1)
        rospy.Subscriber('/debug/stop_state', String, self._cb_stop_state, queue_size=1)
        rospy.Subscriber('/debug/stop_distance', Float64, self._cb_stop_distance, queue_size=1)
        rospy.Subscriber('/debug/stop_dwell_elapsed', Float64, self._cb_dwell_elapsed, queue_size=1)
        rospy.Subscriber('/debug/stop_dwell_remaining', Float64, self._cb_dwell_remaining, queue_size=1)

        self.pub_vehicle = rospy.Publisher('/viz/vehicle_marker',  MarkerArray, queue_size=1)
        self.pub_path    = rospy.Publisher('/viz/path_markers',    MarkerArray, queue_size=1, latch=True)
        self.pub_key     = rospy.Publisher('/viz/key_points',      MarkerArray, queue_size=1, latch=True)
        self.pub_ctrl    = rospy.Publisher('/viz/control_markers', MarkerArray, queue_size=1)
        self.pub_look    = rospy.Publisher('/viz/lookahead_marker', Marker,     queue_size=1)

        self._path_published = False

        rospy.Timer(rospy.Duration(1.0 / self.freq), self._publish)

    def _cb_state(self, msg): self.state = msg

    def _cb_path(self, msg):
        self.path_points = list(msg.points)
        self.scenario_name = msg.scenario_id or self.scenario_name
        self._path_published = False  # force re-publish path markers

    def _cb_scenario(self, msg): self.scenario_name = msg.data

    def _cb_progress(self, msg):
        if self.path_points:
            self.nearest_idx = int(msg.data * max(len(self.path_points) - 1, 1))

    def _cb_speed_ref(self, msg): self.speed_ref = float(msg.data)

    def _cb_stop_state(self, msg): self.stop_state = msg.data

    def _cb_stop_distance(self, msg): self.stop_distance = float(msg.data)

    def _cb_dwell_elapsed(self, msg): self.dwell_elapsed = float(msg.data)

    def _cb_dwell_remaining(self, msg): self.dwell_remaining = float(msg.data)

    def _publish(self, _event):
        if self.state is not None:
            self.pub_vehicle.publish(_fix_quat(self._make_vehicle_markers()))
            self.pub_ctrl.publish(_fix_quat(self._make_control_markers()))
            if self.path_points:
                la = self._make_lookahead_marker()
                if la is not None:
                    self.pub_look.publish(_fix_quat(la))

        if self.path_points and not self._path_published:
            self.pub_path.publish(_fix_quat(self._make_path_markers()))
            self.pub_key.publish(_fix_quat(self._make_key_point_markers()))
            self._path_published = True

    # ------------------------------------------------------------------
    def _make_vehicle_markers(self) -> MarkerArray:
        s = self.state
        ma = MarkerArray()
        hdr = Header(stamp=rospy.Time.now(), frame_id='map')

        # --- heading arrow (kept, does not occlude mesh) ---
        arrow = Marker()
        arrow.header = hdr
        arrow.ns = 'vehicle'; arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points.append(_point(s.x, s.y, 0.5))
        arrow.points.append(_point(
            s.x + 4.0 * np.cos(s.psi),
            s.y + 4.0 * np.sin(s.psi), 0.5))
        arrow.scale = Vector3(x=0.3, y=0.6, z=0.0)
        arrow.color = _color(1.0, 0.9, 0.0)
        arrow.lifetime = rospy.Duration(0.2)
        ma.markers.append(arrow)

        # --- speed text (hovering above mesh) ---
        txt = Marker()
        txt.header = hdr
        txt.ns = 'vehicle'; txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = s.x
        txt.pose.position.y = s.y
        txt.pose.position.z = self._text_speed_z
        txt.scale.z = self._text_speed_size
        txt.color = _color(1.0, 1.0, 1.0)
        txt.text = f"{s.speed * 3.6:.1f} km/h"
        txt.lifetime = rospy.Duration(0.2)
        ma.markers.append(txt)

        # --- status text board beside the vehicle ---
        status = Marker()
        status.header = hdr
        status.ns = 'vehicle'; status.id = 3
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = (
            s.x + self._text_fwd_offset * np.cos(s.psi)
            - self._text_lat_offset * np.sin(s.psi))
        status.pose.position.y = (
            s.y + self._text_fwd_offset * np.sin(s.psi)
            + self._text_lat_offset * np.cos(s.psi))
        status.pose.position.z = self._text_status_z
        status.scale.z = self._text_status_size
        status.color = _color(0.95, 0.95, 0.95, 1.0)
        status.text = self._status_text()
        status.lifetime = rospy.Duration(0.2)
        ma.markers.append(status)

        return ma

    def _status_text(self) -> str:
        phase = self._phase_text()
        actual_kmh = self.state.speed * 3.6 if self.state is not None else float("nan")
        ref_kmh = self.speed_ref * 3.6 if np.isfinite(self.speed_ref) else float("nan")
        progress = self.nearest_idx / max(len(self.path_points) - 1, 1) * 100.0 if self.path_points else 0.0

        lines = [
            f"Scenario: {self.scenario_name}",
            f"Now: {phase}",
            f"Control: {self.stop_state}",
            f"Speed: {actual_kmh:.1f} km/h",
        ]
        if np.isfinite(ref_kmh):
            lines.append(f"Target: {ref_kmh:.1f} km/h")
        if np.isfinite(self.stop_distance):
            lines.append(f"Stop dist: {self.stop_distance:.2f} m")
        if np.isfinite(self.dwell_elapsed):
            lines.append(
                f"Dwell: {self.dwell_elapsed:.1f}s / rem {self.dwell_remaining:.1f}s")
        lines.append(f"Progress: {progress:.1f}%")
        return "\n".join(lines)

    def _phase_text(self) -> str:
        priority_state_map = {
            "brake": "comfort braking",
            "crawl": "precision crawl",
            "settle": "final settling",
            "hold": "station dwell",
            "terminal_hold": "final stop hold",
            "overshot": "overshot stop point",
        }
        if self.stop_state in priority_state_map:
            return priority_state_map[self.stop_state]

        if not self.path_points:
            return "waiting for path"
        idx = min(self.nearest_idx, len(self.path_points) - 1)
        pt = self.path_points[idx]
        if abs(float(getattr(pt, 'kappa', 0.0))) > 1e-4:
            return "curve tracking"
        trend = self._local_speed_trend(idx)
        if trend == "decel":
            return "decelerating to stop"
        if trend == "accel":
            return "accelerating out"
        if np.isfinite(self.stop_distance) and self.stop_distance > 0.0:
            return "cruising to stop"
        return "path tracking"

    def _local_speed_trend(self, idx: int) -> str:
        if not self.path_points:
            return "flat"
        i0 = max(idx - 2, 0)
        i1 = min(idx + 2, len(self.path_points) - 1)
        dv = float(self.path_points[i1].v - self.path_points[i0].v)
        if dv < -0.05:
            return "decel"
        if dv > 0.05:
            return "accel"
        return "flat"

    def _make_path_markers(self) -> MarkerArray:
        ma = MarkerArray()
        if not self.path_points:
            return ma

        # Speed-colored line strip
        pts = self.path_points
        v_max = max(p.v for p in pts) + 0.01

        line = Marker()
        line.header.stamp = rospy.Time.now()
        line.header.frame_id = 'map'
        line.ns = 'path'; line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.25
        line.color = _color(0.0, 1.0, 0.0, 0.8)
        line.lifetime = rospy.Duration(0)  # persistent
        for p in pts:
            line.points.append(_point(p.x, p.y, 0.1))
        ma.markers.append(line)

        # Start / end spheres
        for i, (pt, c) in enumerate([(pts[0], _color(0, 1, 0)),
                                      (pts[-1], _color(1, 0, 0))]):
            sp = Marker()
            sp.header = line.header
            sp.ns = 'path'; sp.id = 1 + i
            sp.type = Marker.SPHERE
            sp.action = Marker.ADD
            sp.pose.position = _point(pt.x, pt.y, 0.5)
            sp.scale = Vector3(x=1.5, y=1.5, z=1.5)
            sp.color = c
            sp.lifetime = rospy.Duration(0)
            ma.markers.append(sp)

        return ma

    def _make_key_point_markers(self) -> MarkerArray:
        ma = MarkerArray()
        pts = self.path_points
        if not pts:
            return ma

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        marker_id = 1

        def add_sphere(pt, label, color, scale=1.5, z=0.8):
            nonlocal marker_id
            m = Marker()
            m.header = header
            m.ns = 'key_points'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = _point(pt.x, pt.y, z)
            m.scale = Vector3(x=scale, y=scale, z=scale)
            m.color = color
            m.lifetime = rospy.Duration(0)
            ma.markers.append(m)

            t = Marker()
            t.header = header
            t.ns = 'key_point_labels'
            t.id = marker_id
            marker_id += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            # Negative lateral offset so labels don't sit directly above the
            # sphere and overlap with the vehicle status text board.
            t.pose.position = _point(pt.x, pt.y - 3.6, z + 1.0)
            t.scale.z = 1.1
            t.color = _color(1.0, 1.0, 1.0, 1.0)
            t.text = label
            t.lifetime = rospy.Duration(0)
            ma.markers.append(t)

        used_indices = set()

        add_sphere(pts[0], "START", _color(0.0, 0.9, 0.2, 1.0), scale=1.8)
        used_indices.add(0)

        stop_indices = [i for i, p in enumerate(pts) if getattr(p, 'stop_point', False)]
        for i in stop_indices:
            label = "FINAL STOP" if i == len(pts) - 1 else "STATION STOP"
            color = _color(1.0, 0.0, 0.0, 1.0) if i == len(pts) - 1 else _color(1.0, 0.7, 0.0, 1.0)
            add_sphere(pts[i], label, color, scale=2.4)
            used_indices.add(i)

        if not stop_indices or stop_indices[-1] != len(pts) - 1:
            end_idx = len(pts) - 1
            if end_idx not in used_indices:
                add_sphere(pts[end_idx], "END", _color(1.0, 0.0, 0.0, 1.0), scale=1.8)
                used_indices.add(end_idx)

        curve_ranges = self._curve_ranges(pts)
        for start, end in curve_ranges:
            if start not in used_indices:
                add_sphere(pts[start], "CURVE IN", _color(0.0, 0.7, 1.0, 1.0), scale=1.4, z=0.6)
                used_indices.add(start)
            if end not in used_indices:
                add_sphere(pts[end], "CURVE OUT", _color(0.0, 0.7, 1.0, 1.0), scale=1.4, z=0.6)
                used_indices.add(end)

        for kind, start, _end in self._speed_change_ranges(pts):
            if start not in used_indices:
                label = "DECEL IN" if kind == "decel" else "ACCEL IN"
                add_sphere(pts[start], label, _color(1.0, 0.35, 0.0, 1.0) if kind == "decel" else _color(0.4, 1.0, 0.0, 1.0), scale=1.2, z=0.5)
                used_indices.add(start)

        return ma

    def _curve_ranges(self, pts):
        ranges = []
        in_curve = False
        start = 0
        threshold = 1e-4
        for i, p in enumerate(pts):
            is_curve = abs(float(getattr(p, 'kappa', 0.0))) > threshold
            if is_curve and not in_curve:
                start = i
                in_curve = True
            elif in_curve and not is_curve:
                end = max(i - 1, start)
                if end > start:
                    ranges.append((start, end))
                in_curve = False
        if in_curve:
            ranges.append((start, len(pts) - 1))
        return ranges

    def _speed_change_ranges(self, pts):
        ranges = []
        if len(pts) < 2:
            return ranges
        current_kind = None
        start = 0
        threshold = 0.02
        for i in range(1, len(pts)):
            dv = float(pts[i].v - pts[i - 1].v)
            if dv < -threshold:
                kind = "decel"
            elif dv > threshold:
                kind = "accel"
            else:
                kind = None

            if kind != current_kind:
                if current_kind is not None and i - 1 > start:
                    ranges.append((current_kind, start, i - 1))
                current_kind = kind
                start = i - 1
        if current_kind is not None and len(pts) - 1 > start:
            ranges.append((current_kind, start, len(pts) - 1))
        return ranges

    def _make_control_markers(self) -> MarkerArray:
        ma = MarkerArray()
        if not self.path_points or self.state is None:
            return ma

        idx = min(self.nearest_idx, len(self.path_points) - 1)
        ref = self.path_points[idx]
        s = self.state

        # Nearest point sphere
        sp = Marker()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.ns = 'ctrl'; sp.id = 0
        sp.type = Marker.SPHERE
        sp.action = Marker.ADD
        sp.pose.position = _point(ref.x, ref.y, 0.3)
        sp.scale = Vector3(x=0.8, y=0.8, z=0.8)
        sp.color = _color(1.0, 0.5, 0.0)
        sp.lifetime = rospy.Duration(0.2)
        ma.markers.append(sp)

        # Lateral error arrow (vehicle → nearest point projection)
        arr = Marker()
        arr.header = sp.header
        arr.ns = 'ctrl'; arr.id = 1
        arr.type = Marker.ARROW
        arr.action = Marker.ADD
        arr.points.append(_point(s.x, s.y, 0.3))
        arr.points.append(_point(ref.x, ref.y, 0.3))
        arr.scale = Vector3(x=0.15, y=0.35, z=0.0)
        arr.color = _color(1.0, 0.2, 0.2, 0.8)
        arr.lifetime = rospy.Duration(0.2)
        ma.markers.append(arr)

        return ma

    def _make_lookahead_marker(self) -> Marker:
        if not self.path_points or self.state is None:
            return None
        vx = max(self.state.v_x, 1.0)
        la_dist = vx * 1.5   # 1.5s lookahead
        s0 = self.path_points[self.nearest_idx].s if self.nearest_idx < len(self.path_points) else 0
        target = None
        for p in self.path_points[self.nearest_idx:]:
            if p.s - s0 >= la_dist:
                target = p
                break
        if target is None:
            target = self.path_points[-1]

        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'map'
        m.ns = 'lookahead'; m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = _point(target.x, target.y, 0.4)
        m.scale = Vector3(x=1.0, y=1.0, z=1.0)
        m.color = _color(0.0, 1.0, 1.0)
        m.lifetime = rospy.Duration(0.2)
        return m


if __name__ == '__main__':
    node = VizNode()
    rospy.spin()
