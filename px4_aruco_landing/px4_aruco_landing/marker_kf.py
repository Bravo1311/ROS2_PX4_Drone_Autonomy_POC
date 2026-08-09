#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class MarkerKalmanFilter:
    """
    Constant-velocity Kalman filter for 3D position.
    State: [x, y, z, vx, vy, vz]^T
    Measurement: [x, y, z]^T
    """

    def __init__(self, q_pos=0.02, q_vel=0.2, r_pos=0.05, p0_pos=1.0, p0_vel=1.0):
        self.x = np.zeros((6, 1), dtype=np.float64)   # state
        self.P = np.zeros((6, 6), dtype=np.float64)   # covariance

        # Initial uncertainty
        self.P[0:3, 0:3] = np.eye(3) * float(p0_pos)
        self.P[3:6, 3:6] = np.eye(3) * float(p0_vel)

        # Measurement model H: picks position from state
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # Measurement noise (camera / ArUco)
        self.R = np.eye(3, dtype=np.float64) * float(r_pos)

        # Process noise strengths (how “non-constant-velocity” you believe motion is)
        self.q_pos = float(q_pos)
        self.q_vel = float(q_vel)

        self.initialized = False

    def _make_A(self, dt: float) -> np.ndarray:
        A = np.eye(6, dtype=np.float64)
        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt
        return A

    def _make_Q(self, dt: float) -> np.ndarray:
        # Simple diagonal Q: allow pos and vel to wander
        Q = np.zeros((6, 6), dtype=np.float64)
        Q[0:3, 0:3] = np.eye(3) * (self.q_pos * dt * dt)
        Q[3:6, 3:6] = np.eye(3) * (self.q_vel * dt)
        return Q

    def predict(self, dt: float):
        A = self._make_A(dt)
        Q = self._make_Q(dt)
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q

    def update(self, z_xyz: np.ndarray):
        z = z_xyz.reshape(3, 1).astype(np.float64)

        y = z - (self.H @ self.x)                       # innovation
        S = self.H @ self.P @ self.H.T + self.R         # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)        # Kalman gain

        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P

    def step(self, z_xyz: np.ndarray, dt: float):
        # On first measurement, initialize position directly (avoid startup transient)
        if not self.initialized:
            self.x[0:3, 0] = z_xyz.reshape(3)
            self.x[3:6, 0] = 0.0
            self.initialized = True
            return

        dt = float(max(1e-3, min(dt, 0.2)))  # clamp dt to sane range
        self.predict(dt)
        self.update(z_xyz)

    def pos(self) -> np.ndarray:
        return self.x[0:3, 0].copy()

    def vel(self) -> np.ndarray:
        return self.x[3:6, 0].copy()


class MarkerPoseKFNode(Node):
    def __init__(self):
        super().__init__("marker_pose_kf")

        # Params
        self.declare_parameter("in_topic", "/marker_pose")
        self.declare_parameter("out_topic", "/marker_pose_filtered")

        # Tune knobs (start here; adjust later)
        self.declare_parameter("r_pos", 0.05)   # measurement noise (ArUco jitter)
        self.declare_parameter("q_pos", 0.02)   # process noise for position
        self.declare_parameter("q_vel", 0.2)    # process noise for velocity

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value

        r_pos = self.get_parameter("r_pos").value
        q_pos = self.get_parameter("q_pos").value
        q_vel = self.get_parameter("q_vel").value

        self.kf = MarkerKalmanFilter(q_pos=q_pos, q_vel=q_vel, r_pos=r_pos)

        self.sub = self.create_subscription(PoseStamped, in_topic, self.cb, 20)
        self.pub = self.create_publisher(PoseStamped, out_topic, 20)

        self._last_stamp = None

        self.get_logger().info(f"Sub: {in_topic}  → Pub: {out_topic}")
        self.get_logger().info(f"Tuning: r_pos={r_pos}, q_pos={q_pos}, q_vel={q_vel}")

    def cb(self, msg: PoseStamped):
        # Compute dt from header stamps
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        if self._last_stamp is None:
            dt = 0.02  # default
        else:
            dt = t - self._last_stamp
        self._last_stamp = t

        z = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)

        self.kf.step(z, dt)
        p = self.kf.pos()

        out = PoseStamped()
        out.header = msg.header  # keep frame_id + timestamp
        out.pose = msg.pose      # copy orientation as-is (we're filtering position only)
        out.pose.position.x = float(p[0])
        out.pose.position.y = float(p[1])
        out.pose.position.z = float(p[2])

        self.pub.publish(out)


def main():
    rclpy.init()
    node = MarkerPoseKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()