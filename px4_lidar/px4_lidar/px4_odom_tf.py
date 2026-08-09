#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np


def quat_wxyz_to_rotmat(qw, qx, qy, qz) -> np.ndarray:
    """Quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    # normalize just in case
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-12:
        return np.eye(3)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n

    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=float)


def rotmat_to_quat_wxyz(R: np.ndarray):
    """3x3 rotation matrix -> quaternion (w,x,y,z)."""
    tr = float(np.trace(R))
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    # normalize output
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    return qw/n, qx/n, qy/n, qz/n


class PX4OdomTF(Node):
    def __init__(self):
        super().__init__('px4_odom_tf')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.cb,
            qos
        )

        # Fixed rotation matrix that maps vectors NED -> ENU:
        # [x_enu, y_enu, z_enu]^T = R_ENU_NED * [x_ned, y_ned, z_ned]^T
        self.R_ENU_NED = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0,-1]
        ], dtype=float)

    def cb(self, msg: VehicleOdometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Position: NED -> ENU
        x_ned, y_ned, z_ned = msg.position
        t.transform.translation.x = float(y_ned)
        t.transform.translation.y = float(x_ned)
        t.transform.translation.z = float(-z_ned)

        # PX4 q is [w, x, y, z]
        qw, qx, qy, qz = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])

        R_ned_frd = quat_wxyz_to_rotmat(qw, qx, qy, qz)

        R_enu_ned = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0,-1]
        ], dtype=float)

        # Convert body convention FRD -> FLU (right-multiply)
        R_frd_flu = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ], dtype=float)

        # Correct transform: ENU<-NED applied on left, FRD<-FLU applied on right
        R_enu_flu = R_enu_ned @ R_ned_frd @ R_frd_flu

        qw2, qx2, qy2, qz2 = rotmat_to_quat_wxyz(R_enu_flu)

        t.transform.rotation.w = float(qw2)
        t.transform.rotation.x = float(qx2)
        t.transform.rotation.y = float(qy2)
        t.transform.rotation.z = float(qz2)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = PX4OdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()