#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np


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

    def cb(self, msg: VehicleOdometry):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # PX4 uses NED, ROS uses ENU
        # Convert NED → ENU
        # Convert NED → ENU (position)
        x_ned, y_ned, z_ned = msg.position
        t.transform.translation.x = float(y_ned)
        t.transform.translation.y = float(x_ned)
        t.transform.translation.z = float(-z_ned)

        # Quaternion (still NED→ENU placeholder for now)
        q = msg.q  # [w, x, y, z]

        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[2])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(-q[3])

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = PX4OdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()