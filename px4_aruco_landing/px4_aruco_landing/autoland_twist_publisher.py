#!/usr/bin/env python3
import time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


def clamp(v, lo, hi):
    return max(lo, min(hi, v))  

class AutoLandTwist(Node):
    """
    Reads marker pose in camera_optical_frame and produces a BODY-FRAME Twist
    (forward x, left y, up z) for the PX4OffboardMux to forward to PX4.

    Publishes:
      - /autoland_velocity_cmd (Twist)
      - /autoland_enable (Bool)  (optional; you can also control enable from CLI)
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
        self.declare_parameter("k_xy", 0.6)
        self.declare_parameter("vmax_xy", 0.8)
        self.declare_parameter("descend_rate", 0.6)      # m/s DOWN in body frame -> that's NEGATIVE z (since up is +z)
        self.declare_parameter("center_radius", 0.25)
        self.declare_parameter("lost_timeout_s", 0.6)
        self.declare_parameter("kd_xy", 0.25)
        self.declare_parameter("d_filter_alpha", 0.7)  # 0..1, higher = smoother derivative

        self.kd_xy = float(self.get_parameter("kd_xy").value)
        self.alpha = float(self.get_parameter("d_filter_alpha").value)

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_t = None
        self.dex_f = 0.0
        self.dey_f = 0.0

        self.marker_topic = self.get_parameter("marker_pose_topic").value
        self.k_xy = float(self.get_parameter("k_xy").value)
        self.vmax_xy = float(self.get_parameter("vmax_xy").value)
        self.descend_rate = float(self.get_parameter("descend_rate").value)
        self.center_radius = float(self.get_parameter("center_radius").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)

        # State
        self.last_marker_time = None
        self.ex = 0.0
        self.ey = 0.0
        self.marker_seen = False

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, "/autoland_velocity_cmd", qos)

        self.create_subscription(PoseStamped, self.marker_topic, self.on_marker_pose, 10)

        self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info(f"AutoLandTwist listening: {self.marker_topic}")

    def on_marker_pose(self, msg: PoseStamped):
        # msg.pose.position is in camera_optical_frame:
        #   x: right, y: down, z: forward (out of lens)
        #
        # For a DOWN-facing camera, a common practical mapping for ground-plane errors is:
        #   body forward (+x)  ~  -optical y
        #   body left   (+y)  ~  -optical x   (because optical x is right)
        #
        # If you see the drone moving away, flip signs one at a time here.
        body_ex_forward = -float(msg.pose.position.y)
        # body_ey_left    = -float(msg.pose.position.x)
        body_ey_left    = float(msg.pose.position.x)

        self.ex = body_ex_forward
        self.ey = body_ey_left
        self.marker_seen = True
        self.last_marker_time = time.time()

    def loop(self):
        now = time.time()

        # Decide enable based on user param AND marker freshness

        # marker freshness
        if self.last_marker_time is None or (now - self.last_marker_time) > self.lost_timeout:
            self.marker_seen = False

        out = Twist()

        if not self.marker_seen:
            # stop if marker lost
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.z = 0.0
            self.cmd_pub.publish(out)
            self.get_logger().warn("Marker lost -> autoland outputs zero", throttle_duration_sec=1.0)
            return

        e = math.sqrt(self.ex*self.ex + self.ey*self.ey)
        t = time.time()

        if self.prev_t is None:
            dt = 1.0 / 20.0   # assume loop rate on first iteration
        else:
            dt = t - self.prev_t

        dt = max(0.02, min(0.2, dt))   # clamp dt safely
        dex = (self.ex - self.prev_ex) / dt
        dey = (self.ey - self.prev_ey) / dt

        DEX_MAX = 2.0  # m/s equivalent, tune
        DEY_MAX = 2.0
        dex = max(-DEX_MAX, min(DEX_MAX, dex))
        dey = max(-DEY_MAX, min(DEY_MAX, dey))

        # low-pass derivative (very important)
        self.dex_f = self.alpha * self.dex_f + (1.0 - self.alpha) * dex
        self.dey_f = self.alpha * self.dey_f + (1.0 - self.alpha) * dey

        self.prev_ex, self.prev_ey, self.prev_t = self.ex, self.ey, t

        vx = clamp(self.k_xy * self.ex + self.kd_xy * self.dex_f, -self.vmax_xy, self.vmax_xy)
        vy = clamp(self.k_xy * self.ey + self.kd_xy * self.dey_f, -self.vmax_xy, self.vmax_xy)

        # Descend only when centered
        if e < self.center_radius:
            # In BODY frame we define +z = UP, so descending is NEGATIVE z
            vz = abs(self.descend_rate)
        else:
            vz = 0.0

        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = vz
        out.angular.z = 0.0

        self.cmd_pub.publish(out)

        self.get_logger().info(
            f"AUTO cmd (body): vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} err={e:.2f}",
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
