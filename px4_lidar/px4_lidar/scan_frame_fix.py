#!/usr/bin/env python3

# Copyright 2026 Kartik Agrawal
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ScanFrameFix(Node):
    def __init__(self):
        super().__init__("scan_frame_fix")

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.declare_parameter("in_topic", "/scan")
        self.declare_parameter("out_topic", "/scan_fixed")
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("assumed_scan_rate_hz", 10.0)  # used only if scan_time==0

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.assumed_rate = float(self.get_parameter("assumed_scan_rate_hz").value)

        # Sensor QoS on BOTH ends (BEST_EFFORT + VOLATILE)
        self.pub = self.create_publisher(LaserScan, self.out_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.in_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Rewriting LaserScan frame_id -> '{self.frame_id}'  ({self.in_topic} -> {self.out_topic})"
        )

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.frame_id

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities

        # Patch missing timing if simulator/bridge provides 0.0
        if out.scan_time == 0.0 and self.assumed_rate > 0.0:
            out.scan_time = 1.0 / self.assumed_rate
        if out.time_increment == 0.0 and len(out.ranges) > 1 and out.scan_time > 0.0:
            out.time_increment = out.scan_time / float(len(out.ranges) - 1)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ScanFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()