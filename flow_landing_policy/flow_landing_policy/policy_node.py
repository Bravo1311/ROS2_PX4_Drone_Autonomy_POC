import time
import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist

from flow_landing_policy.inference import load_policy, generate_action_chunk
from flow_landing_policy.config import *

def marker_pose_to_relative_pos_quat(msg: PoseStamped):
    """
        Converts an incoming ArUco PoseStamped (camera_optical_frame: +X right, +Y down, +Z forward) into the (relative_pos, relative_quat) convention the flow-matching policy was trained on in MuJoCo.

        Note: Policy convention: 
            pos_err = marker_pos - drone_pos
            drone yaw conventional
            for flow matching, z needs to be negative
    """
    # w.r.t drone's body frame
    x = -msg.pose.position.y
    y = msg.pose.position.x
    z = -msg.pose.position.z

    relative_pos = np.array([x, y, z], dtype = np.float32)
    q = msg.pose.orientation
    relative_quat = np.array([q.w, q.x, q.y, q.z], dtype = np.float32)

    return relative_pos, relative_quat

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class FlowLandingPolicyNode(Node):
    """
        Similar to Aruco-detector landing implementation, implements 
    """

    def __init__(self):
        super().__init__("flow_landing_policy")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ---------------- Params ----------------
        self.declare_parameter("marker_pose_topic", "marker_pose")
        self.declare_parameter("weights_path", "")
        self.declare_parameter("steps_per_chunk", 3)
        self.declare_parameter("lost_timeout_s", 0.6)
        self.declare_parameter("vmax_xy", 1.0)
        self.declare_parameter("vmax_z", 1.0)
        self.declare_parameter("max_yaw_rate", 0.8)

        self.marker_topic = self.get_parameter("marker_pose_topic").value
        weights_path = self.get_parameter("weights_path").value
        self.steps_per_chunk = int(self.get_parameter("steps_per_chunk").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)
        self.vmax_xy = float(self.get_parameter("vmax_xy").value)
        self.vmax_z = float(self.get_parameter("vmax_z").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        if not weights_path:
            raise ValueError("weights_path parameter is required")

        # ---------------- Model ----------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Loading policy on device: {self.device}")
        self.policy = load_policy(weights_path, self.device)
        self.get_logger().info("Policy loaded.")

        # ---------------- State ----------------
        self.history_buffer = []       # rolling list of (7,) pose vectors
        self.current_chunk = None       # (CHUNK_LEN, 4) generated actions
        self.chunk_idx = 0
        self.steps_since_replan = self.steps_per_chunk  # force replan on first call

        self.last_marker_time = None
        self.marker_seen = False

        # ---------------- Pub/Sub ----------------
        # self.cmd_pub = self.create_publisher(Twist, "/autoland_velocity_cmd", qos)
        self.cmd_pub = self.create_publisher(Twist, "/autoland_velocity_cmd", qos)
        self.create_subscription(PoseStamped, self.marker_topic, self.on_marker_pose, 10)
        self.create_timer(0.05, self.loop)  # 20 Hz, matches AutoLandTwist cadence

        self.get_logger().info(
            f"FlowLandingPolicyNode listening: {self.marker_topic} | "
            f"steps_per_chunk={self.steps_per_chunk}"
        )

    def reset_state(self):
        self.history_buffer = []
        self.current_chunk = None
        self.chunk_idx = 0
        self.steps_since_replan = self.steps_per_chunk

    def on_marker_pose(self, msg: PoseStamped):
        relative_pos, relative_quat = marker_pose_to_relative_pos_quat(msg)
        pose_vec = np.concatenate([relative_pos, relative_quat])  # (7,)

        self.history_buffer.append(pose_vec)
        if len(self.history_buffer) > HISTORY_LEN:
            self.history_buffer.pop(0)

        self.marker_seen = True
        self.last_marker_time = time.time()

    def loop(self):
        now = time.time()

        if self.last_marker_time is None or (now - self.last_marker_time) > self.lost_timeout:
            if self.marker_seen:
                self.get_logger().warn("Marker lost -> outputs zero + reset policy state")
            self.marker_seen = False

        out = Twist()

        if not self.marker_seen or len(self.history_buffer) < HISTORY_LEN:
            # not enough real history yet, or marker not visible: hold still
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.z = 0.0
            self.cmd_pub.publish(out)
            if not self.marker_seen:
                self.reset_state()
            return

        # ---------------- replan if needed ----------------
        if self.steps_since_replan >= self.steps_per_chunk:
            history_arr = np.stack(self.history_buffer[-HISTORY_LEN:])  # (H, 7)
            self.current_chunk = generate_action_chunk(self.policy, history_arr, self.device)
            self.chunk_idx = 0
            self.steps_since_replan = 0

        cmd_vel = self.current_chunk[self.chunk_idx]
        self.chunk_idx += 1
        self.steps_since_replan += 1

        vx, vy, vz, yaw_rate = cmd_vel

        # ---------------- safety clamps ----------------
        vx = clamp(float(vx), -self.vmax_xy, self.vmax_xy)
        vy = clamp(float(vy), -self.vmax_xy, self.vmax_xy)
        vz = clamp(float(vz), -self.vmax_z, self.vmax_z)
        yaw_rate = clamp(float(yaw_rate), -self.max_yaw_rate, self.max_yaw_rate)

        out.linear.x = vx
        out.linear.y = vy

        # to convert mujoco convention vel_z to px4-gz
        out.linear.z = -vz
        out.angular.z = yaw_rate

        self.cmd_pub.publish(out)

        self.get_logger().info(
            f"POLICY: vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw={yaw_rate:.2f} "
            f"chunk_idx={self.chunk_idx-1}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = FlowLandingPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()