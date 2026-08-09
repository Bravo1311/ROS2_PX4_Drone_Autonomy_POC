#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StartupSpin(Node):
    """
    Publishes a yaw-rate Twist for a short duration to help AMCL converge.
    Publish topic should feed into your existing cmd_vel pipeline.
    """
    def __init__(self):
        super().__init__("startup_spin")

        # Params
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_raw")
        self.declare_parameter("yaw_rate", 0.5)      # rad/s
        self.declare_parameter("duration", 8.0)      # seconds
        self.declare_parameter("publish_hz", 10.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.duration = float(self.get_parameter("duration").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.tick)

        self.get_logger().info(
            f"Startup spin publishing to {self.cmd_vel_topic} for {self.duration}s at yaw_rate={self.yaw_rate}"
        )

    def tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        msg = Twist()

        if elapsed <= self.duration:
            msg.angular.z = self.yaw_rate
            self.pub.publish(msg)
            return

        # Stop
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info("Startup spin done. Stopping publisher.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = StartupSpin()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()