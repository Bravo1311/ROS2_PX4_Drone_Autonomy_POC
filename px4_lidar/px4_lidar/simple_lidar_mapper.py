#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """
    Bresenham line algorithm: returns grid cells from (x0,y0) to (x1,y1).
    """
    cells = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    cells.append((x1, y1))
    return cells


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Convert quaternion to yaw.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleLidarMapper(Node):
    def __init__(self):
        super().__init__("simple_lidar_mapper")

        # ---------------- Parameters ----------------
        self.declare_parameter("scan_topic", "/scan_fixed")
        self.declare_parameter("map_topic", "/simple_map")

        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")

        self.declare_parameter("resolution", 0.05)   # m/cell
        self.declare_parameter("width", 400)         # cells
        self.declare_parameter("height", 400)        # cells
        self.declare_parameter("origin_x", -10.0)    # meters
        self.declare_parameter("origin_y", -10.0)    # meters

        self.declare_parameter("log_odds_occ", 0.85)
        self.declare_parameter("log_odds_free", -0.4)
        self.declare_parameter("log_odds_min", -5.0)
        self.declare_parameter("log_odds_max", 5.0)

        self.declare_parameter("publish_rate", 2.0)  # Hz
        self.declare_parameter("max_range_clip", 8.0)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value

        self.resolution = float(self.get_parameter("resolution").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)

        self.log_odds_occ = float(self.get_parameter("log_odds_occ").value)
        self.log_odds_free = float(self.get_parameter("log_odds_free").value)
        self.log_odds_min = float(self.get_parameter("log_odds_min").value)
        self.log_odds_max = float(self.get_parameter("log_odds_max").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.max_range_clip = float(self.get_parameter("max_range_clip").value)

        # ---------------- Map storage ----------------
        # unknown = 0.0 log-odds initially
        self.log_odds = [0.0] * (self.width * self.height)

        # ---------------- TF ----------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- ROS interfaces ----------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_map
        )

        self.get_logger().info("Simple LiDAR mapper started.")
        self.get_logger().info(
            f"Map: {self.width}x{self.height}, res={self.resolution}, "
            f"origin=({self.origin_x}, {self.origin_y})"
        )

    # ---------------------------------------------------
    # Grid helpers
    # ---------------------------------------------------
    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def grid_to_index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def update_log_odds(self, gx: int, gy: int, delta: float):
        if not self.in_bounds(gx, gy):
            return
        idx = self.grid_to_index(gx, gy)
        self.log_odds[idx] = max(
            self.log_odds_min,
            min(self.log_odds_max, self.log_odds[idx] + delta)
        )

    # ---------------------------------------------------
    # TF
    # ---------------------------------------------------
    def lookup_robot_pose(self) -> Tuple[float, float, float]:
        """
        Returns robot pose in map frame: x, y, yaw
        """
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            raise RuntimeError(str(e))

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        yaw = quat_to_yaw(qx, qy, qz, qw)
        return tx, ty, yaw

    # ---------------------------------------------------
    # Scan callback
    # ---------------------------------------------------
    def scan_callback(self, scan: LaserScan):
        try:
            robot_x, robot_y, robot_yaw = self.lookup_robot_pose()
        except RuntimeError as e:
            self.get_logger().warn(f"TF unavailable: {e}", throttle_duration_sec=2.0)
            return

        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        if not self.in_bounds(robot_gx, robot_gy):
            self.get_logger().warn("Robot pose outside map bounds.", throttle_duration_sec=2.0)
            return

        angle = scan.angle_min

        for r in scan.ranges:
            # Skip invalid
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            if r < scan.range_min:
                angle += scan.angle_increment
                continue

            hit = True
            if r > scan.range_max:
                r = scan.range_max
                hit = False

            if r > self.max_range_clip:
                r = self.max_range_clip
                hit = False

            beam_yaw = robot_yaw + angle
            end_x = robot_x + r * math.cos(beam_yaw)
            end_y = robot_y + r * math.sin(beam_yaw)

            end_gx, end_gy = self.world_to_grid(end_x, end_y)

            if not self.in_bounds(end_gx, end_gy):
                angle += scan.angle_increment
                continue

            cells = bresenham(robot_gx, robot_gy, end_gx, end_gy)

            # Mark free space along ray except endpoint
            for cell in cells[:-1]:
                self.update_log_odds(cell[0], cell[1], self.log_odds_free)

            # Mark endpoint occupied only if actual hit
            if hit:
                self.update_log_odds(end_gx, end_gy, self.log_odds_occ)

            angle += scan.angle_increment

    # ---------------------------------------------------
    # Publish map
    # ---------------------------------------------------
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for lo in self.log_odds:
            # unknown-ish around zero
            if abs(lo) < 1e-3:
                data.append(-1)
            else:
                p = 1.0 - 1.0 / (1.0 + math.exp(lo))  # logistic
                occ = int(max(0, min(100, round(p * 100.0))))
                data.append(occ)

        msg.data = data
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()