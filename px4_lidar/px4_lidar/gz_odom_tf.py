#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped


class GzOdomTF(Node):
    def __init__(self):
        super().__init__("gz_odom_tf")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.declare_parameter("odom_topic", "/model/x500_mono_cam_down_0/odometry_with_covariance")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")

        odom_topic = self.get_parameter("odom_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, qos)

        self.get_logger().info(f"Publishing TF: {self.parent_frame} -> {self.child_frame} from {odom_topic}")

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()   # <-- change this
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GzOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()