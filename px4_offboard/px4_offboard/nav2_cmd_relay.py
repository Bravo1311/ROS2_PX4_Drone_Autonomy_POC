#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class Nav2CmdRelay(Node):
    """
    /cmd_vel_nav  ->  /autoland_velocity_cmd
    - Forces z = 0 (Nav2 is 2D)
    - Optional clamps
    - Watchdog stop on timeout
    """
    def __init__(self):
        super().__init__("nav2_cmd_relay")

        self.declare_parameter("in_topic", "/cmd_vel_nav")
        self.declare_parameter("out_topic", "/nav_velocity_cmd")
        self.declare_parameter("timeout_s", 0.4)
        self.declare_parameter("max_vx", 1.0)
        self.declare_parameter("max_vy", 1.0)
        self.declare_parameter("max_wz", 1.0)

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)

        self.last_rx = self.get_clock().now()
        self.last_cmd = Twist()

        self.timer = self.create_timer(0.05, self.watchdog)  # 20 Hz
        self.get_logger().info(f"Relay: {self.in_topic} -> {self.out_topic} (z forced 0)")

    def cb(self, msg: Twist):
        cmd = Twist()
        cmd.linear.x = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(msg.linear.y, -self.max_vy, self.max_vy)
        cmd.linear.z = 0.0  # <<< THE IMPORTANT PART
        cmd.angular.z = clamp(msg.angular.z, -self.max_wz, self.max_wz)

        self.last_cmd = cmd
        self.last_rx = self.get_clock().now()
        self.pub.publish(cmd)

    def watchdog(self):
        age = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        if age > self.timeout_s:
            self.pub.publish(Twist())  # stop

def main():
    rclpy.init()
    n = Nav2CmdRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()