#!/usr/bin/env python3
# px4_offboard_mux_with_safety.py
#
# QoS contract:
#   - Teleop inputs (/cmd_vel, /offboard_velocity_cmd, /autoland_velocity_cmd): BEST_EFFORT (teleop compatible)
#   - Internal chain (/cmd_vel_raw, /cmd_vel_safe): RELIABLE (robust + echo-friendly)
#   - PX4 topics: your existing BEST_EFFORT + TRANSIENT_LOCAL pattern

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy,
    qos_profile_sensor_data
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleAttitude
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


SETPOINT_RATE_HZ = 50.0
SETPOINT_WARMUP_COUNT = 30
INACTIVITY_TIMEOUT = 5.0

VEL_LIMIT_XY = 3.0
VEL_LIMIT_Z = 2.0
YAW_RATE_LIMIT = 2.0

SAFE_TIMEOUT_S = 1.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PX4OffboardMux(Node):
    def __init__(self):
        super().__init__("px4_offboard_mux")

        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Teleop QoS (compatible with most joystick publishers)
        qos_teleop = qos_profile_sensor_data  # BEST_EFFORT

        # Internal cmd chain QoS (RELIABLE)
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # PX4 pubs
        self.ctrl_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_px4)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_px4)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_px4)

        # Internal chain
        self.cmd_raw_pub = self.create_publisher(Twist, "/cmd_vel_raw", qos_cmd)

        # PX4 subs
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.status_cb, qos_px4)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.att_cb, qos_px4)

        # Teleop/auto subs (subscribe to BOTH teleop topics to avoid remap headaches)
        self.create_subscription(Twist, "/cmd_vel", self.teleop_cb, qos_teleop)
        self.create_subscription(Twist, "/offboard_velocity_cmd", self.teleop_cb, qos_teleop)
        self.create_subscription(Twist, "/autoland_velocity_cmd", self.auto_cb, qos_teleop)

        self.create_subscription(Bool, "/enable_auto_land", self.enable_auto_cb, qos_px4)
        self.create_subscription(Bool, "/arm_message", self.arm_cb, qos_px4)

        # Safety output (RELIABLE)
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe_cb, qos_cmd)

        self.status = VehicleStatus()
        self.offboard_active = False
        self.armed = False
        self.warmed_up = False

        self.yaw = 0.0

        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()
        self.last_teleop_t = 0.0
        self.last_auto_t = 0.0
        self.auto_enabled = False

        self.safe_cmd = Twist()
        self.last_safe_t = 0.0

        self.create_timer(1.0 / SETPOINT_RATE_HZ, self.loop)
        self.get_logger().info("PX4 Offboard Mux (with safety) initialized.")

    def status_cb(self, msg: VehicleStatus):
        self.status = msg

    def att_cb(self, msg: VehicleAttitude):
        q = msg.q
        if len(q) == 4:
            w, x, y, z = q
            self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def safe_cb(self, msg: Twist):
        self.safe_cmd = msg
        self.last_safe_t = time.time()

    def teleop_cb(self, msg: Twist):
        self.teleop_cmd = msg
        self.last_teleop_t = time.time()
        # self.get_logger().warn("TELEOP RX", throttle_duration_sec=0.5)
        if not self.warmed_up and self.armed:
            self.start_offboard()

    def auto_cb(self, msg: Twist):
        self.auto_cmd = msg
        self.last_auto_t = time.time()
        if not self.warmed_up and self.armed:
            self.start_offboard()

    def enable_auto_cb(self, msg: Bool):
        self.auto_enabled = bool(msg.data)

    def arm_cb(self, msg: Bool):
        if msg.data and not self.armed:
            self.get_logger().info("ARM command received")
            self.start_offboard()
            time.sleep(0.2)
            self.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.armed = True
            self.get_logger().info("ARMED - Ready to fly!")
        elif (not msg.data) and self.armed:
            self.get_logger().info("DISARM command received")
            self.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.armed = False
            self.warmed_up = False
            self.offboard_active = False
            self.teleop_cmd = Twist()
            self.auto_cmd = Twist()
            self.safe_cmd = Twist()
            self.last_safe_t = 0.0
            self.get_logger().info("DISARMED")

    def micros(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.micros()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.ctrl_mode_pub.publish(msg)

    def publish_setpoint_world(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = self.micros()
        msg.position = [float("nan")] * 3
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = float("nan")
        msg.yawspeed = float(yawspeed)
        self.setpoint_pub.publish(msg)

    def publish_cmd(self, cmd, **params):
        m = VehicleCommand()
        m.timestamp = self.micros()
        m.command = int(cmd)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        for k in ["param1", "param2", "param3", "param4", "param5", "param6", "param7"]:
            setattr(m, k, float(params.get(k, 0.0)))
        self.cmd_pub.publish(m)

    def start_offboard(self):
        if self.warmed_up:
            return

        self.get_logger().info("Warming up Offboard mode...")
        for _ in range(SETPOINT_WARMUP_COUNT):
            self.publish_mode()
            self.publish_setpoint_world(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0 / SETPOINT_RATE_HZ)

        self.get_logger().info("Requesting OFFBOARD mode...")
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.warmed_up = True
        time.sleep(0.2)
        self.get_logger().info("Offboard mode ready!")

    def select_cmd_body(self):
        now = time.time()
        teleop_fresh = (now - self.last_teleop_t) <= INACTIVITY_TIMEOUT
        auto_fresh = (now - self.last_auto_t) <= INACTIVITY_TIMEOUT

        if self.auto_enabled and auto_fresh:
            src = "AUTO"
            cmd = self.auto_cmd
            fresh = True
        elif teleop_fresh:
            src = "TELEOP"
            cmd = self.teleop_cmd
            fresh = True
        else:
            src = "IDLE"
            cmd = Twist()
            fresh = False

        out = Twist()
        out.linear.x = clamp(cmd.linear.x, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        out.linear.y = clamp(cmd.linear.y, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        out.linear.z = clamp(cmd.linear.z, -VEL_LIMIT_Z, VEL_LIMIT_Z)
        out.angular.z = clamp(cmd.angular.z, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)
        return src, out, fresh

    def loop(self):
        self.publish_mode()
        self.offboard_active = (self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        src, body_raw, fresh = self.select_cmd_body()

        # ALWAYS publish raw (RELIABLE) so safety always sees it
        self.cmd_raw_pub.publish(body_raw if fresh else Twist())

        # Don’t fly unless offboard + armed
        if (not self.armed) or (not self.offboard_active):
            self.publish_setpoint_world(0.0, 0.0, 0.0, 0.0)
            return

        if not fresh:
            self.publish_setpoint_world(0.0, 0.0, 0.0, 0.0)
            return

        safe_fresh = (time.time() - self.last_safe_t) <= SAFE_TIMEOUT_S
        if not safe_fresh:
            self.get_logger().warn("cmd_vel_safe stale -> hover", throttle_duration_sec=1.0)
            self.publish_setpoint_world(0.0, 0.0, 0.0, 0.0)
            return

        vx_b = self.safe_cmd.linear.x
        vy_b = self.safe_cmd.linear.y
        vz_b = self.safe_cmd.linear.z
        yawspeed = self.safe_cmd.angular.z

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        wx = vx_b * cy - vy_b * sy
        wy = vx_b * sy + vy_b * cy
        wz = vz_b

        self.publish_setpoint_world(wx, wy, wz, yawspeed)

        self.get_logger().info(
            f"[{src}] body_safe vx={vx_b:.2f} vy={vy_b:.2f} vz={vz_b:.2f} yr={yawspeed:.2f} | "
            f"world vx={wx:.2f} vy={wy:.2f} vz={wz:.2f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PX4 Offboard Mux node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()