#!/usr/bin/env python3
import time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class AutoLandTwist(Node):
    """
    Reads marker pose in camera_optical_frame and produces a BODY-FRAME Twist
    (forward x, left y, up z) for the PX4OffboardMux to forward to PX4.

    Publishes:
      - /autoland_velocity_cmd (Twist)
    """

    def __init__(self):
        super().__init__("autoland_twist")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Params
        self.declare_parameter("marker_pose_topic", "marker_pose")

        # P
        self.declare_parameter("k_xy", 0.5)
        self.declare_parameter("vmax_xy", 0.8)

        # I
        self.declare_parameter("ki_xy", 0.00)           # integral gain
        self.declare_parameter("i_limit", 0.5)          # clamp integral state (meters*sec)
        self.declare_parameter("i_leak", 0.0)           # 0..1 per second, optional (0 = no leak)

        # D
        self.declare_parameter("kd_xy", 0.1)         # 0.35
        self.declare_parameter("d_filter_alpha", 0.7)   # 0..1, higher = smoother derivative

        # Landing logic
        self.declare_parameter("descend_rate", 0.7)     # magnitude m/s (down). We will output NEGATIVE z.
        self.declare_parameter("center_radius", 0.40)
        self.declare_parameter("lost_timeout_s", 0.6)

        # Load params
        self.marker_topic = self.get_parameter("marker_pose_topic").value
 
        self.k_xy = float(self.get_parameter("k_xy").value)             # Kp
        self.vmax_xy = float(self.get_parameter("vmax_xy").value)

        self.ki_xy = float(self.get_parameter("ki_xy").value)           # KI
        self.i_limit = float(self.get_parameter("i_limit").value)       # Integral clamp: Limits how big the integral can be
        self.i_leak = float(self.get_parameter("i_leak").value)         # reduces accumulated integral over time
 
        self.kd_xy = float(self.get_parameter("kd_xy").value)           # Kd
        self.alpha = float(self.get_parameter("d_filter_alpha").value)  # low-pass filter on the derivative

        self.descend_rate = float(self.get_parameter("descend_rate").value)
        self.center_radius = float(self.get_parameter("center_radius").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)

        # State
        self.last_marker_time = None
        self.marker_seen = False

        self.ex = 0.0
        self.ey = 0.0
        self.marker_z = 0.0  # camera optical z (out of lens)

        # D state
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_t = None
        self.dex_f = 0.0
        self.dey_f = 0.0

        # I state
        self.ix = 0.0
        self.iy = 0.0

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, "/autoland_velocity_cmd", qos)
        self.create_subscription(PoseStamped, self.marker_topic, self.on_marker_pose, 10)
        self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info(f"AutoLandTwist(PID) listening: {self.marker_topic}")

    def reset_pid_states(self):
        # Reset I and D states (useful when marker is lost)
        self.ix = 0.0
        self.iy = 0.0
        self.dex_f = 0.0
        self.dey_f = 0.0
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_t = None

    def on_marker_pose(self, msg: PoseStamped):
        # camera_optical_frame:
        #   x: right, y: down, z: forward
        #
        # For down-facing camera mapping to body:
        #   body forward (+x) ~ -optical y
        #   body left    (+y) ~  optical x  (optical x is right -> left is negative, but you chose + here)
        body_ex_forward = -float(msg.pose.position.y)
        body_ey_left = float(msg.pose.position.x)   # flip if needed

        self.marker_z = float(msg.pose.position.z) 
        z = max(self.marker_z, 0.5)          # meters, avoid division blow-ups
        self.ex = body_ex_forward / z
        self.ey = body_ey_left / z 

        self.marker_seen = True
        self.last_marker_time = time.time()

    def loop(self):
        now = time.time()

        # marker freshness check
        if self.last_marker_time is None or (now - self.last_marker_time) > self.lost_timeout:
            if self.marker_seen:
                self.get_logger().warn("Marker lost -> autoland outputs zero + reset PID")
            self.marker_seen = False

        out = Twist()

        if not self.marker_seen:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.z = 0.0
            self.cmd_pub.publish(out)

            # IMPORTANT: avoid integral windup while marker is missing
            self.reset_pid_states()
            return

        # timing
        t = time.time()
        if self.prev_t is None:
            dt = 1.0 / 20.0
        else:
            dt = t - self.prev_t
        dt = max(0.02, min(0.2, dt))  # clamp

        # --- D term (filtered derivative of error) ---
        dex = (self.ex - self.prev_ex) / dt
        dey = (self.ey - self.prev_ey) / dt

        DEX_MAX = 2.0
        DEY_MAX = 2.0
        dex = clamp(dex, -DEX_MAX, DEX_MAX)
        dey = clamp(dey, -DEY_MAX, DEY_MAX)

        self.dex_f = self.alpha * self.dex_f + (1.0 - self.alpha) * dex
        self.dey_f = self.alpha * self.dey_f + (1.0 - self.alpha) * dey

        # --- PID x/y with anti-windup (conditional integration) ---
        # Compute unsaturated output first (P + I + D)
        ux_unsat = self.k_xy * self.ex + self.ki_xy * self.ix + self.kd_xy * self.dex_f
        uy_unsat = self.k_xy * self.ey + self.ki_xy * self.iy + self.kd_xy * self.dey_f

        vx = clamp(ux_unsat, -self.vmax_xy, self.vmax_xy)
        vy = clamp(uy_unsat, -self.vmax_xy, self.vmax_xy)

        # Conditional integration:
        # Integrate if not saturated, OR if error would drive output back toward unsaturation.
        sat_x = (vx != ux_unsat)
        sat_y = (vy != uy_unsat)

        if (not sat_x) or (sat_x and (ux_unsat * self.ex) < 0.0):
            self.ix += self.ex * dt

        if (not sat_y) or (sat_y and (uy_unsat * self.ey) < 0.0):
            self.iy += self.ey * dt

        # Optional leak (helps if you get slow drift / bias)
        if self.i_leak > 0.0:
            leak = clamp(self.i_leak * dt, 0.0, 1.0)
            self.ix *= (1.0 - leak)
            self.iy *= (1.0 - leak)

        # Clamp integral state (hard anti-windup bound)
        self.ix = clamp(self.ix, -self.i_limit, self.i_limit)
        self.iy = clamp(self.iy, -self.i_limit, self.i_limit)

        # Update prevs
        self.prev_ex, self.prev_ey, self.prev_t = self.ex, self.ey, t

        # Recompute vx, vy after updated integrator (optional but cleaner)
        vx = clamp(self.k_xy * self.ex + self.ki_xy * self.ix + self.kd_xy * self.dex_f,
                   -self.vmax_xy, self.vmax_xy)
        vy = clamp(self.k_xy * self.ey + self.ki_xy * self.iy + self.kd_xy * self.dey_f,
                   -self.vmax_xy, self.vmax_xy)

        # Descend only when centered
        e = math.sqrt(self.ex * self.ex + self.ey * self.ey)
        if e < self.center_radius:
            # BODY frame: +z = UP, so descending is NEGATIVE z
            vz = abs(self.descend_rate)
        else:
            vz = 0.0

        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = vz
        out.angular.z = 0.0
        self.cmd_pub.publish(out)

        self.get_logger().info(
            f"AUTO PID (body): vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} "
            f"err={e:.2f} ix={self.ix:.2f} iy={self.iy:.2f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoLandTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()