#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AutoLandOffboard(Node):
    """
    Subscribes:
      - marker_pose (PoseStamped) from ArUco node (tvec in camera optical frame, meters)
      - /fmu/out/vehicle_local_position
      - /fmu/out/vehicle_status

    Publishes:
      - /fmu/in/offboard_control_mode
      - /fmu/in/trajectory_setpoint   (velocity setpoints)
      - /fmu/in/vehicle_command       (arm, set mode offboard, land)
    """

    def __init__(self):
        super().__init__("auto_land_offboard")

        # --- Params ---
        self.declare_parameter("marker_pose_topic", "marker_pose")

        # Controller gains and limits
        self.declare_parameter("k_xy", 0.6)             # proportional gain
        self.declare_parameter("vmax_xy", 1.0)          # m/s
        self.declare_parameter("descend_rate", 0.4)     # m/s downward in NED (positive down)
        self.declare_parameter("center_radius", 0.25)   # m; start descending when within this radius
        self.declare_parameter("lost_timeout_s", 0.6)

        # Behavior
        self.declare_parameter("auto_arm_offboard", True)
        self.declare_parameter("send_land_below_m", 0.25)  # send LAND when below this altitude and centered

        self.marker_topic = self.get_parameter("marker_pose_topic").value
        self.k_xy = float(self.get_parameter("k_xy").value)
        self.vmax_xy = float(self.get_parameter("vmax_xy").value)
        self.descend_rate = float(self.get_parameter("descend_rate").value)
        self.center_radius = float(self.get_parameter("center_radius").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)
        self.auto_arm_offboard = bool(self.get_parameter("auto_arm_offboard").value)
        self.land_below = float(self.get_parameter("send_land_below_m").value)

        # State
        self.last_marker_time = None
        self.ex = 0.0
        self.ey = 0.0
        self.marker_seen = False

        self.local_pos = None
        self.status = None

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Subscribers
        self.create_subscription(PoseStamped, self.marker_topic, self.on_marker_pose, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_local_pos, 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_status, 10)

        # Timer (setpoint stream)
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.start_time = time.time()
        self.sent_offboard = False
        self.sent_arm = False
        self.sent_land = False

        self.get_logger().info(f"Auto-landing listening marker pose: {self.marker_topic}")

    def on_marker_pose(self, msg: PoseStamped):
        # tvec in OpenCV optical frame:
        #   x: right, y: down, z: forward (for down cam, forward ~ down)
        # We need lateral error in *some* consistent frame for commanding vx/vy.
        #
        # Practical mapping for many down-facing cams:
        #   - camera x (right) corresponds to body right
        #   - camera y (down in image) corresponds to body forward/back depending on mounting
        #
        # We'll use a simple mapping that works in many setups:
        #   ex_forward = -msg.pose.position.y
        #   ey_right   =  msg.pose.position.x
        #
        # If signs are flipped, you’ll see it immediately: drone moves away instead of toward.
        # Then flip sign(s).

        ex_forward = -float(msg.pose.position.y)
        ey_right = float(msg.pose.position.x)

        self.ex = ex_forward
        self.ey = ey_right
        self.marker_seen = True
        self.last_marker_time = time.time()

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.local_pos = msg
        self.get_logger().info(f"The local position is:  {self.local_pos}")

    def on_status(self, msg: VehicleStatus):
        self.status = msg

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, yawspeed=0.0):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # --- Velocity fields differ between px4_msgs versions ---
        if hasattr(sp, "vx") and hasattr(sp, "vy") and hasattr(sp, "vz"):
            # older layout
            sp.vx = float(vx)
            sp.vy = float(vy)
            sp.vz = float(vz)
        elif hasattr(sp, "velocity"):
            # newer layout: float32[3] velocity
            sp.velocity = [float(vx), float(vy), float(vz)]
        else:
            raise AttributeError(
                "TrajectorySetpoint has neither (vx,vy,vz) nor velocity[3]. "
                "Run: ros2 interface show px4_msgs/msg/TrajectorySetpoint"
            )

        # yaw/yawspeed fields also differ
        if hasattr(sp, "yawspeed"):
            sp.yawspeed = float(yawspeed)
        elif hasattr(sp, "yaw_rate"):
            sp.yaw_rate = float(yawspeed)

        self.setpoint_pub.publish(sp)


    def on_timer(self):
        now = time.time()

        # Always publish offboard control mode + setpoints continuously
        self.publish_offboard_mode()

        # Marker freshness
        if self.last_marker_time is None or (now - self.last_marker_time) > self.lost_timeout:
            self.marker_seen = False

        # Optionally arm + go offboard (only do after streaming setpoints for a moment)
        if self.auto_arm_offboard and not self.sent_offboard and (now - self.start_time) > 1.0:
            # Set mode to Offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)  # 6 = offboard in PX4 convention
            self.sent_offboard = True

        if self.auto_arm_offboard and self.sent_offboard and not self.sent_arm and (now - self.start_time) > 1.5:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.sent_arm = True

        # Compute commands
        if not self.marker_seen:
            # If marker not seen: stop (safe default)
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("Marker not received. Exiting")
            return

        e = math.sqrt(self.ex * self.ex + self.ey * self.ey)

        vx = clamp(self.k_xy * self.ex, -self.vmax_xy, self.vmax_xy)
        vy = clamp(self.k_xy * self.ey, -self.vmax_xy, self.vmax_xy)

        # descend only when centered
        if e < self.center_radius:
            vz = self.descend_rate  # NED: positive down
        else:
            vz = 0.0

        self.publish_velocity_setpoint(vx, vy, vz)

        # Optional: send LAND when very close to ground and centered
        if self.local_pos is not None and not self.sent_land and e < (0.6 * self.center_radius):
            # In PX4 local position, z is usually Down (NED). Altitude above origin is -z.
            # If origin is at takeoff and you took off, local_pos.z is positive down when below origin.
            # For landing decision, easiest: use dist-to-ground from marker tvec z if you trust it.
            # Here we use camera distance proxy: msg.pose.position.z isn't stored, so skip.
            #
            # We'll use local_pos.z magnitude only as a crude check:
            if abs(float(self.local_pos.z)) < self.land_below:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.sent_land = True
                self.get_logger().info("Sent LAND command.")


def main():
    rclpy.init()
    node = AutoLandOffboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
