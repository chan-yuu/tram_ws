#!/usr/bin/env python3
"""ROS data logger — records key experiment topics to CSV.

Auto-shutdown when progress >= 0.99 and speed < 0.1 m/s for 3 seconds.
Usage:
  python3 data_logger.py /path/to/output.csv
"""
import sys
import csv
import rospy
from std_msgs.msg import Float64, String
from tram_msgs.msg import VehicleState


class DataLogger:
    def __init__(self, output_path: str):
        rospy.init_node('data_logger', anonymous=False)
        self.output_path = output_path
        self.csv_file = open(output_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time', 'scenario',
            'x', 'y', 'psi', 'vx', 'vy', 'r', 'speed',
            'speed_actual', 'speed_ref',
            'lateral_error', 'heading_error',
            'steering_front', 'steering_rear',
            'accel_cmd', 'progress',
        ])

        self.data = {k: 0.0 for k in [
            'x', 'y', 'psi', 'vx', 'vy', 'r', 'speed',
            'speed_actual', 'speed_ref',
            'lateral_error', 'heading_error',
            'steering_front', 'steering_rear',
            'accel_cmd', 'progress',
        ]}
        self.data['scenario'] = ''
        self._stop_start = None

        rospy.Subscriber('/planning/scenario', String, self._cb_scenario)
        rospy.Subscriber('/sim/vehicle_state', VehicleState, self._cb_state)
        rospy.Subscriber('/debug/speed_actual', Float64, self._cb('speed_actual'))
        rospy.Subscriber('/debug/speed_ref', Float64, self._cb('speed_ref'))
        rospy.Subscriber('/debug/lateral_error', Float64, self._cb('lateral_error'))
        rospy.Subscriber('/debug/heading_error', Float64, self._cb('heading_error'))
        rospy.Subscriber('/debug/steering_front', Float64, self._cb('steering_front'))
        rospy.Subscriber('/debug/steering_rear', Float64, self._cb('steering_rear'))
        rospy.Subscriber('/debug/accel_cmd', Float64, self._cb('accel_cmd'))
        rospy.Subscriber('/debug/progress', Float64, self._cb('progress'))

        self.timer = rospy.Timer(rospy.Duration(0.05), self._record)  # 20 Hz
        rospy.loginfo("[data_logger] logging to %s", output_path)

    def _cb(self, key):
        def cb(msg):
            self.data[key] = msg.data
        return cb

    def _cb_scenario(self, msg):
        self.data['scenario'] = msg.data

    def _cb_state(self, msg):
        self.data['x'] = msg.x
        self.data['y'] = msg.y
        self.data['psi'] = msg.psi
        self.data['vx'] = msg.v_x
        self.data['vy'] = msg.v_y
        self.data['r'] = msg.r
        self.data['speed'] = msg.speed

    def _record(self, _event):
        t = rospy.Time.now().to_sec()
        d = self.data
        self.writer.writerow([
            t, d['scenario'],
            d['x'], d['y'], d['psi'], d['vx'], d['vy'], d['r'], d['speed'],
            d['speed_actual'], d['speed_ref'],
            d['lateral_error'], d['heading_error'],
            d['steering_front'], d['steering_rear'],
            d['accel_cmd'], d['progress'],
        ])

        # Auto-detect completion
        if d['progress'] >= 0.99 and d['speed'] < 0.1:
            if self._stop_start is None:
                self._stop_start = rospy.Time.now()
            elif (rospy.Time.now() - self._stop_start).to_sec() > 3.0:
                rospy.loginfo("[data_logger] scenario complete, shutting down.")
                rospy.signal_shutdown("completed")
        else:
            self._stop_start = None

    def run(self):
        rospy.spin()
        self.csv_file.close()
        rospy.loginfo("[data_logger] saved %s", self.output_path)


if __name__ == '__main__':
    output = sys.argv[1] if len(sys.argv) > 1 else '/tmp/tram_data.csv'
    logger = DataLogger(output)
    logger.run()
