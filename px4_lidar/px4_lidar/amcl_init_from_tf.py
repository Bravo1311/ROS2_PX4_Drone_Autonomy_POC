#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class AMCLInitFromTF(Node):
    def __init__(self):
        super().__init__("amcl_init_from_tf")

        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("initialpose_frame", "map")
        self.declare_parameter("publish_count", 10)
        self.declare_parameter("publish_period", 0.2)
        self.declare_parameter("xy_cov", 0.04)   # 0.2 m std
        self.declare_parameter("yaw_cov", 0.02)  # ~8 deg std

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.initialpose_frame = self.get_parameter("initialpose_frame").value
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.xy_cov = float(self.get_parameter("xy_cov").value)
        self.yaw_cov = float(self.get_parameter("yaw_cov").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sent = 0
        self.timer = self.create_timer(self.publish_period, self.tick)

        self.get_logger().info(
            f"Will publish /initialpose in '{self.initialpose_frame}' using TF {self.parent_frame}->{self.child_frame}"
        )

    def tick(self):
        if self.sent >= self.publish_count:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.initialpose_frame

        msg.pose.pose.position.x = float(t.transform.translation.x)
        msg.pose.pose.position.y = float(t.transform.translation.y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = t.transform.rotation

        cov = [0.0] * 36
        cov[0] = self.xy_cov
        cov[7] = self.xy_cov
        cov[35] = self.yaw_cov
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.sent += 1

        if self.sent == 1:
            self.get_logger().info(
                f"Published initialpose x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f} (x{self.publish_count})"
            )
        if self.sent >= self.publish_count:
            self.get_logger().info("Done publishing /initialpose.")
            # Keep alive; shutting down can race AMCL startup.


def main():
    rclpy.init()
    node = AMCLInitFromTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()