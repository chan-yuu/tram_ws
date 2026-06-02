#!/usr/bin/env python3
"""Path planning node — generates reference paths and publishes them.

Publishes:
  /planning/path        tram_msgs/TramPath  (latched)
  /planning/path_viz    nav_msgs/Path       (latched, for RViz)
  /planning/scenario    std_msgs/String
  /planning/target_speed  std_msgs/Float64  (1 Hz, current ref speed)
"""
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import numpy as np

from tram_msgs.msg import TramPath, PathPoint as RosPathPoint
from tram_planning.scenarios.standard_stop  import StandardStop
from tram_planning.scenarios.highspeed_stop import HighspeedStop
from tram_planning.scenarios.short_platform  import ShortPlatform
from tram_planning.scenarios.switch_lane     import SwitchLane
from tram_planning.scenarios.uturn_180       import UTurn180

SCENARIO_MAP = {
    'standard_stop':  StandardStop,
    'highspeed_stop': HighspeedStop,
    'short_platform': ShortPlatform,
    'switch_lane':    SwitchLane,
    'uturn_180':      UTurn180,
}


def _quat_from_yaw(yaw):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class PlanningNode:
    def __init__(self):
        rospy.init_node('planning_node', anonymous=False)

        scenario_name = rospy.get_param('~scenario', 'standard_stop')
        self.ds = rospy.get_param('~ds', 0.5)

        # --- publishers (latched for path) ---
        self.pub_tram   = rospy.Publisher('/planning/path',      TramPath, queue_size=1, latch=True)
        self.pub_viz    = rospy.Publisher('/planning/path_viz',  Path,     queue_size=1, latch=True)
        self.pub_scen   = rospy.Publisher('/planning/scenario',  String,   queue_size=1, latch=True)
        self.pub_speed  = rospy.Publisher('/planning/target_speed', Float64, queue_size=1)

        self.waypoints = []
        self._load_scenario(scenario_name)

        # 1 Hz diagnostics
        rospy.Timer(rospy.Duration(1.0), self._publish_speed)

        rospy.loginfo("[planning_node] scenario='%s', %d points",
                      scenario_name, len(self.waypoints))

    # ------------------------------------------------------------------
    def _load_scenario(self, name: str):
        if name not in SCENARIO_MAP:
            rospy.logwarn("[planning_node] unknown scenario '%s', using 'standard_stop'", name)
            name = 'standard_stop'

        scenario = SCENARIO_MAP[name]()
        self.waypoints = scenario.generate(ds=self.ds)
        self.scenario_name = name
        self._publish_path()

    def _publish_path(self):
        now = rospy.Time.now()

        # --- TramPath (custom, latched) ---
        tram_msg = TramPath()
        tram_msg.header.stamp = now
        tram_msg.header.frame_id = 'map'
        tram_msg.scenario_id = self.scenario_name
        for wp in self.waypoints:
            pt = RosPathPoint()
            pt.x = wp.x; pt.y = wp.y; pt.yaw = wp.yaw
            pt.v = wp.v; pt.s = wp.s; pt.kappa = wp.kappa
            pt.stop_point = wp.stop_point
            pt.dwell_time = wp.dwell_time
            tram_msg.points.append(pt)
        self.pub_tram.publish(tram_msg)

        # --- nav_msgs/Path (standard, latched) ---
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'map'
        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'map'
            ps.pose.position.x = wp.x
            ps.pose.position.y = wp.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = _quat_from_yaw(wp.yaw)
            path_msg.poses.append(ps)
        self.pub_viz.publish(path_msg)

        # --- scenario name ---
        self.pub_scen.publish(String(data=self.scenario_name))

    def _publish_speed(self, _event):
        if not self.waypoints:
            return
        # Publish mid-path reference speed as representative value
        mid = self.waypoints[len(self.waypoints) // 2]
        self.pub_speed.publish(Float64(data=mid.v))


if __name__ == '__main__':
    node = PlanningNode()
    rospy.spin()
