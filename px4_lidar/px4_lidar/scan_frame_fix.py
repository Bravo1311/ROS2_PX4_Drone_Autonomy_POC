#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class ScanFrameFix(Node):
    """
    Subscribes to a LaserScan topic and republishes it with a new header.frame_id.
    Default:
      in_topic  = /scan
      out_topic = /scan_fixed
      frame_id  = lidar_link
    """
    def __init__(self):
        super().__init__("scan_frame_fix")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.declare_parameter("in_topic", "/scan")
        self.declare_parameter("out_topic", "/scan_fixed")
        self.declare_parameter("frame_id", "lidar_link")

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(LaserScan, self.out_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.in_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Rewriting LaserScan frame_id -> '{self.frame_id}'  ({self.in_topic} -> {self.out_topic})"
        )

    def cb(self, msg: LaserScan):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()