#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy,
    qos_profile_sensor_data
)

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SafetyVelFilter(Node):
    def __init__(self):
        super().__init__("safety_vel_filter")

        # Put timer + subs in a reentrant group so they can run freely
        self.cb_group = ReentrantCallbackGroup()

        # RELIABLE QoS for internal cmd chain (must match mux)
        self.qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Params
        self.declare_parameter("cmd_in", "/cmd_vel_raw")
        self.declare_parameter("cmd_out", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan_fixed")

        self.declare_parameter("stop_dist", 0.1)
        self.declare_parameter("slow_dist", 2.5)
        self.declare_parameter("fov_deg", 60.0)

        self.declare_parameter("scan_timeout_s", 1.0)
        self.declare_parameter("stop_gain", 0.5)
        self.declare_parameter("slow_gain", 0.8)
        self.declare_parameter("max_yaw_rate", 1.2)

        self.cmd_in = self.get_parameter("cmd_in").value
        self.cmd_out = self.get_parameter("cmd_out").value
        self.scan_topic = self.get_parameter("scan_topic").value

        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.slow_dist = float(self.get_parameter("slow_dist").value)
        self.fov = math.radians(float(self.get_parameter("fov_deg").value))
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)

        self.stop_gain = float(self.get_parameter("stop_gain").value)
        self.slow_gain = float(self.get_parameter("slow_gain").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.last_scan = None
        self.last_scan_time = None
        self.last_cmd = Twist()

        # Subscriptions
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.on_scan,
            qos_profile_sensor_data,
            callback_group=self.cb_group
        )

        self.sub_cmd = self.create_subscription(
            Twist,
            self.cmd_in,
            self.on_cmd,
            self.qos_cmd,
            callback_group=self.cb_group
        )

        # Publisher
        self.pub_cmd = self.create_publisher(
            Twist,
            self.cmd_out,
            self.qos_cmd
        )

        # Timer at 50 Hz
        self.timer = self.create_timer(
            1.0 / 50.0,
            self.tick,
            callback_group=self.cb_group
        )

        # Heartbeat timer at 1 Hz (to prove timers are executing)
        # self.heartbeat = self.create_timer(
        #     1.0,
        #     self.heartbeat_cb,
        #     callback_group=self.cb_group
        # )

        self.get_logger().info(f"SafetyVelFilter up: {self.cmd_in} + {self.scan_topic} -> {self.cmd_out}")

    def heartbeat_cb(self):
        scan_age = None
        if self.last_scan_time is not None:
            now = self.get_clock().now()
            scan_age = (now - self.last_scan_time).nanoseconds * 1e-9
        self.get_logger().warn(
            f"HEARTBEAT: timer alive. scan_age={scan_age}, last_cmd=({self.last_cmd.linear.x:.2f}, {self.last_cmd.linear.y:.2f}, {self.last_cmd.linear.z:.2f})",
            throttle_duration_sec=0.0
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def on_cmd(self, msg: Twist):
        # self.get_logger().warn(
        #     f"on_cmd: lin=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
        #     f"ang_z={msg.angular.z:.2f}",
        #     throttle_duration_sec=0.5
        # )
        self.last_cmd = msg

    def sector_min_dist(self, scan: LaserScan, center_angle: float) -> float:
        if scan is None or not scan.ranges:
            return float("inf")

        a0 = scan.angle_min
        da = scan.angle_increment
        n = len(scan.ranges)

        half = self.fov / 2.0
        left = center_angle + half
        right = center_angle - half

        i_min = int(math.floor((right - a0) / da))
        i_max = int(math.ceil((left - a0) / da))

        i_min = max(0, min(n - 1, i_min))
        i_max = max(0, min(n - 1, i_max))
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        dmin = float("inf")
        rmin_valid = scan.range_min
        rmax_valid = scan.range_max

        for i in range(i_min, i_max + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and rmin_valid <= r <= rmax_valid:
                dmin = min(dmin, r)
        self.get_logger().info(f'dmin is: {dmin}', throttle_duration_sec=2.0)
        return dmin

    def tick(self):
        # Always run; publish something every tick
        out = Twist()
        out.linear.x = self.last_cmd.linear.x
        out.linear.y = self.last_cmd.linear.y
        out.linear.z = self.last_cmd.linear.z
        out.angular.z = clamp(self.last_cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        vx, vy = out.linear.x, out.linear.y
        vxy = math.hypot(vx, vy)

        # If scan stale -> stop XY+yaw
        now = self.get_clock().now()
        if self.last_scan_time is None or (now - self.last_scan_time).nanoseconds * 1e-9 > self.scan_timeout:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0
            self.pub_cmd.publish(out)
            return

        if vxy > 0.05:
            heading = math.atan2(vy, vx)
            dmin = self.sector_min_dist(self.last_scan, heading)

            stop = self.stop_dist + self.stop_gain * vxy
            slow = self.slow_dist + self.slow_gain * vxy
            if slow <= stop + 1e-3:
                slow = stop + 1e-3

            if dmin < stop:
                out.linear.x = 0.0
                out.linear.y = 0.0
            elif dmin < slow:
                s = (dmin - stop) / (slow - stop)
                s = clamp(s, 0.0, 1.0)
                out.linear.x *= s
                out.linear.y *= s

        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = SafetyVelFilter()

    # MultiThreadedExecutor prevents timer starvation
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()