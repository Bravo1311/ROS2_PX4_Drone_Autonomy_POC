#!/usr/bin/env python3
"""
px4_offboard_mux_working.py

Working PX4 offboard teleop node + mux switch.

Inputs:
  /offboard_velocity_cmd   (Twist)  teleop (body-frame commands)
  /autoland_velocity_cmd   (Twist)  auto   (body-frame commands)
  /enable_auto_land        (Bool)   when True, AUTO has priority if fresh
  /arm_message             (Bool)   arm/disarm trigger (kept identical behavior to your working code)

Behavior:
  - Uses your warmup loop with sleeps (because your setup works with it)
  - Uses your attitude->yaw and rotates (body FLU) -> world frame velocities
  - AUTO overrides TELEOP when enabled & fresh
  - If neither source fresh -> hover (0 velocities)
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleAttitude
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ---- Tunables (kept from your working code) ----
SETPOINT_RATE_HZ = 50.0
SETPOINT_WARMUP_COUNT = 30

# In your working code this was 5s; keep it.
INACTIVITY_TIMEOUT = 5.0  # seconds

VEL_LIMIT_XY = 3.0
VEL_LIMIT_Z = 2.0
YAW_RATE_LIMIT = 2.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PX4OffboardMux(Node):
    def __init__(self):
        super().__init__('px4_offboard_mux_working')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.ctrl_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers (PX4)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos)

        # Subscribers (commands)
        self.create_subscription(Twist, '/offboard_velocity_cmd', self.teleop_cb, qos)
        self.create_subscription(Twist, '/autoland_velocity_cmd', self.auto_cb, qos)
        self.create_subscription(Bool, '/enable_auto_land', self.enable_auto_cb, qos)
        self.create_subscription(Bool, '/arm_message', self.arm_cb, qos)

        # PX4 state
        self.status = VehicleStatus()
        self.offboard_active = False
        self.armed = False

        # Attitude/yaw
        self.yaw = 0.0  # current yaw from PX4 attitude quaternion

        # Command storage (BODY-FRAME inputs)
        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()
        self.last_teleop_t = 0.0
        self.last_auto_t = 0.0

        self.auto_enabled = False

        # Offboard state
        self.warmed_up = False

        # Timer
        self.create_timer(1.0 / SETPOINT_RATE_HZ, self.loop)
        self.get_logger().info("PX4 Offboard Mux (working-base) initialized.")

    # -------------------- Callbacks --------------------
    def status_cb(self, msg: VehicleStatus):
        self.status = msg

    def att_cb(self, msg: VehicleAttitude):
        q = msg.q
        if len(q) == 4:
            w, x, y, z = q
            self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def teleop_cb(self, msg: Twist):
        # store raw; clamp later in selection to keep behavior consistent for both sources
        self.teleop_cmd = msg
        self.last_teleop_t = time.time()

        # your original behavior: if armed and not warmed up, start offboard
        if not self.warmed_up and self.armed:
            self.start_offboard()

    def auto_cb(self, msg: Twist):
        self.auto_cmd = msg
        self.last_auto_t = time.time()

        # also allow auto stream to kick off offboard if already armed
        if not self.warmed_up and self.armed:
            self.start_offboard()

    def enable_auto_cb(self, msg: Bool):
        self.auto_enabled = bool(msg.data)

    def arm_cb(self, msg: Bool):
        """
        Kept deliberately close to your working code behavior,
        even though it "assumes armed" after sending the command.
        """
        if msg.data and not self.armed:
            self.get_logger().info("ARM command received")
            self.start_offboard()  # Prepare offboard mode
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

            # clear commands
            self.teleop_cmd = Twist()
            self.auto_cmd = Twist()
            self.get_logger().info("DISARMED")

    # -------------------- Helpers --------------------
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

    def publish_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = self.micros()
        msg.position = [float('nan')] * 3
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')       # using yawspeed
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
        """Send dummy setpoints before activating OFFBOARD (kept identical pattern)."""
        if self.warmed_up:
            return

        self.get_logger().info("Warming up Offboard mode...")
        for _ in range(SETPOINT_WARMUP_COUNT):
            self.publish_mode()
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(1.0 / SETPOINT_RATE_HZ)

        self.get_logger().info("Requesting OFFBOARD mode...")
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.warmed_up = True
        time.sleep(0.2)
        self.get_logger().info("Offboard mode ready!")

    # -------------------- Mux selection --------------------
    def select_cmd_body(self):
        """
        Returns: (src, vx, vy, vz, yawspeed, fresh)
        vx,vy,vz,yawspeed are BODY-FRAME (your convention), clamped.
        """
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

        vx = clamp(cmd.linear.x, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        vy = clamp(cmd.linear.y, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        vz = clamp(cmd.linear.z, -VEL_LIMIT_Z, VEL_LIMIT_Z)
        yawspeed = clamp(cmd.angular.z, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)

        return src, vx, vy, vz, yawspeed, fresh

    # -------------------- Main Loop --------------------
    def loop(self):
        self.publish_mode()

        # offboard active check (same as your working code)
        if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard_active = True

        # If not Offboard or not armed, hold still
        if not self.offboard_active or not self.armed:
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Select muxed command (BODY frame)
        src, vx_b, vy_b, vz_b, yawspeed, fresh = self.select_cmd_body()

        self.get_logger().info(
            f"auto_enabled={self.auto_enabled} auto_age={time.time()-self.last_auto_t:.2f}s teleop_age={time.time()-self.last_teleop_t:.2f}s",
            throttle_duration_sec=0.5
        )

        # If nothing fresh, hover (and warn occasionally)
        if not fresh:
            self.get_logger().warn(
                "No input (teleop/auto) - entering hover failsafe...",
                throttle_duration_sec=2.0
            )
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Rotate BODY -> WORLD (kept exactly like your working code)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        wx = vx_b * cy - vy_b * sy
        wy = vx_b * sy + vy_b * cy
        wz = vz_b

        self.publish_setpoint(wx, wy, wz, yawspeed)

        # Light debug
        self.get_logger().info(
            f"[{src}] sp: vx={wx:.2f}, vy={wy:.2f}, vz={wz:.2f}, yawspeed={yawspeed:.2f}",
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


if __name__ == '__main__':
    main()