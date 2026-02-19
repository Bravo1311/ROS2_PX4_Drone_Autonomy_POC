#!/usr/bin/env python3
"""
px4_offboard_mux.py

Single authority node that:
- Streams OFFBOARD setpoints continuously
- Accepts velocity commands from:
    /offboard_velocity_cmd   (teleop)   geometry_msgs/Twist
    /autoland_velocity_cmd   (auto)     geometry_msgs/Twist
  and selects one (auto has priority when enabled & fresh)
- Arms/Disarms from:
    /arm_message             std_msgs/Bool  (True=arm, False=disarm)
- (Optional) enables auto source via:
    /enable_auto_land        std_msgs/Bool

Key fixes vs your current version:
- NO sleeping inside callbacks
- DO NOT set self.armed = True just because you sent an arm command
  (armed is derived from PX4 vehicle_status)
- Warmup/offboard/arm is a timer-driven state machine so setpoint streaming never pauses
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

# --- Tunables ---
SETPOINT_RATE_HZ = 50.0
SETPOINT_WARMUP_COUNT = 30          # send neutral setpoints for this many cycles before requesting offboard
INACTIVITY_TIMEOUT = 0.5            # seconds: if cmd older than this => treat as inactive

VEL_LIMIT_XY = 3.0
VEL_LIMIT_Z = 2.0
YAW_RATE_LIMIT = 2.0

# PX4 custom mode: OFFBOARD is usually "6" in PX4 examples via VEHICLE_CMD_DO_SET_MODE param2
PX4_MAIN_MODE_MANUAL = 1.0          # param1 for VEHICLE_CMD_DO_SET_MODE (PX4 uses 1 for custom main mode)
PX4_CUSTOM_SUBMODE_OFFBOARD = 6.0   # param2


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PX4OffboardMux(Node):
    def __init__(self):
        super().__init__("px4_offboard_mux")

        # ---- QoS ----
        # PX4 /fmu/in publishers: safest is TRANSIENT_LOCAL + BEST_EFFORT
        self.qos_px4_in = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # PX4 /fmu/out subscribers: VOLATILE + BEST_EFFORT is typical
        self.qos_px4_out = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Internal command topics: VOLATILE is fine
        self.qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Publishers to PX4 ----
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_px4_in
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.qos_px4_in
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", self.qos_px4_in
        )

        # ---- Subscribers from PX4 ----
        self.status = VehicleStatus()
        self.offboard_active = False
        self.armed = False

        # NOTE: you used *_v1 in your setup; keep it consistent with what your bridge publishes.
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self.status_cb, self.qos_px4_out
        )

        # ---- Internal command inputs ----
        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()
        self.last_teleop_t = 0.0
        self.last_auto_t = 0.0

        self.auto_enabled = False
        self.arm_request = False
        self.disarm_request = False

        self.create_subscription(
            Twist, "/offboard_velocity_cmd", self.teleop_cb, self.qos_cmd
        )
        self.create_subscription(
            Twist, "/autoland_velocity_cmd", self.autoland_cb, self.qos_cmd
        )
        self.create_subscription(
            Bool, "/enable_auto_land", self.enable_auto_cb, self.qos_cmd
        )
        self.create_subscription(
            Bool, "/arm_message", self.arm_cb, self.qos_cmd
        )

        # ---- Offboard/arming state machine ----
        self.warmup_i = 0
        self.warmed_up = False
        self.sent_offboard_request = False

        # ---- Timer loop ----
        period = 1.0 / SETPOINT_RATE_HZ
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info("PX4OffboardMux started (streams setpoints continuously).")

    # ---------------- Callbacks ----------------
    def status_cb(self, msg: VehicleStatus):
        self.status = msg
        # IMPORTANT: derive truth from PX4, don't guess
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def teleop_cb(self, msg: Twist):
        self.teleop_cmd = msg
        self.last_teleop_t = time.time()

    def autoland_cb(self, msg: Twist):
        self.auto_cmd = msg
        self.last_auto_t = time.time()

    def enable_auto_cb(self, msg: Bool):
        self.auto_enabled = bool(msg.data)

    def arm_cb(self, msg: Bool):
        # NO sleeps, no warmup here. Just set intent flags.
        if msg.data:
            self.get_logger().info("ARM requested")
            self.arm_request = True
            self.disarm_request = False
        if self.armed:
            self.arm_request = False


    # ---------------- PX4 publish helpers ----------------
    def publish_offboard_mode(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        m = OffboardControlMode()
        m.timestamp = now_us

        # We control velocity + yaw rate
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False

        self.offboard_mode_pub.publish(m)

    def publish_setpoint(self, vx: float, vy: float, vz: float, yaw_rate: float):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us

        # Velocity setpoints.
        # NOTE: keep same convention your old working node used.
        sp.vx = float(vx)
        sp.vy = float(vy)
        sp.vz = float(vz)

        # PX4 expects yaw in radians (absolute). If you were using yaw_rate previously,
        # you likely controlled yaw via VehicleCommand or used yaw as "current + rate*dt".
        # Here we store yaw as NaN to indicate "don’t care", and use yaw_rate via vehicle command is not typical.
        # Many setups just ignore yaw and steer with vx/vy; keep yaw NaN to avoid conflicts.
        sp.yaw = float("nan")
        sp.yawspeed = float(yaw_rate)

        self.setpoint_pub.publish(sp)

    def publish_cmd(self, command: int, **params):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        c = VehicleCommand()
        c.timestamp = now_us
        c.param1 = float(params.get("param1", 0.0))
        c.param2 = float(params.get("param2", 0.0))
        c.param3 = float(params.get("param3", 0.0))
        c.param4 = float(params.get("param4", 0.0))
        c.param5 = float(params.get("param5", 0.0))
        c.param6 = float(params.get("param6", 0.0))
        c.param7 = float(params.get("param7", 0.0))

        c.command = int(command)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True

        self.cmd_pub.publish(c)

    # ---------------- Command selection ----------------
    def select_cmd(self):
        now = time.time()
        teleop_fresh = (now - self.last_teleop_t) < INACTIVITY_TIMEOUT
        auto_fresh = (now - self.last_auto_t) < INACTIVITY_TIMEOUT

        if self.auto_enabled and auto_fresh:
            src = "AUTO"
            cmd = self.auto_cmd
        elif teleop_fresh:
            src = "TELEOP"
            cmd = self.teleop_cmd
        else:
            src = "IDLE"
            cmd = Twist()

        # Clamp
        vx = clamp(cmd.linear.x, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        vy = clamp(cmd.linear.y, -VEL_LIMIT_XY, VEL_LIMIT_XY)
        vz = clamp(cmd.linear.z, -VEL_LIMIT_Z, VEL_LIMIT_Z)
        yaw_rate = clamp(cmd.angular.z, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)

        return src, vx, vy, vz, yaw_rate

    # ---------------- Main loop ----------------
    def loop(self):
        # Always stream offboard control mode at a steady rate
        self.publish_offboard_mode()

        # DISARM request: send once, reset state machine
        if self.disarm_request:
            if self.armed:
                self.publish_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0,
                )
            self.disarm_request = False
            self.arm_request = False
            self.warmup_i = 0
            self.warmed_up = False
            self.sent_offboard_request = False

            # keep streaming neutral setpoints
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # ARM flow: warmup -> request offboard -> arm retries (no sleeping)
        if self.arm_request and not self.armed:
            # 1) warmup neutral setpoints
            if self.warmup_i < SETPOINT_WARMUP_COUNT:
                self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
                self.warmup_i += 1
                return

            # 2) request OFFBOARD once (or a couple of times if you like)
            if not self.sent_offboard_request:
                self.publish_cmd(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=PX4_MAIN_MODE_MANUAL,
                    param2=PX4_CUSTOM_SUBMODE_OFFBOARD,
                )
                self.sent_offboard_request = True
                # keep neutral setpoint this cycle too
                self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
                return

            # 3) keep trying to arm until PX4 reports armed
            self.publish_cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # If not armed yet (and not in arm flow), hold neutral
        if not self.armed:
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Armed: publish selected command setpoint
        src, vx, vy, vz, yaw_rate = self.select_cmd()
        self.publish_setpoint(vx, vy, vz, yaw_rate)


def main():
    rclpy.init()
    node = PX4OffboardMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
