#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


class GzOdomTF(Node):
    def __init__(self):
        super().__init__('gz_odom_tf')

        self.declare_parameter('model_name',   'x500_mono_cam_down_0')
        self.declare_parameter('gz_topic',     '/world/walls/dynamic_pose/info')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame',  'base_link')

        self.model_name   = self.get_parameter('model_name').value
        self.gz_topic     = self.get_parameter('gz_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame  = self.get_parameter('child_frame').value

        # Shared state
        self.latest_pose  = None
        self.latest_stamp = None
        self.lock = threading.Lock()

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Odometry publisher — same data, different format, for Nav2
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ROS timer at 50Hz
        self.create_timer(1.0 / 50.0, self._publish)

        # gz.transport subscriber
        self.gz_node = GzNode()
        self.gz_node.subscribe(Pose_V, self.gz_topic, self._gz_cb)

        self.get_logger().info(
            f'gz_odom_tf: listening for "{self.model_name}" on {self.gz_topic}'
        )

    def _gz_cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name == self.model_name:
                with self.lock:
                    self.latest_pose  = pose
                    self.latest_stamp = msg.header.stamp
                return

    def _publish(self):
        with self.lock:
            pose  = self.latest_pose
            stamp = self.latest_stamp

        if pose is None:
            return

        # Build the stamp once, reuse in both messages
        ros_stamp = self.get_clock().now().to_msg()
        ros_stamp.sec     = stamp.sec
        ros_stamp.nanosec = stamp.nsec

        # ── 1. TF ──────────────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp    = ros_stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x    = pose.orientation.x
        t.transform.rotation.y    = pose.orientation.y
        t.transform.rotation.z    = pose.orientation.z
        t.transform.rotation.w    = pose.orientation.w

        self.br.sendTransform(t)

        # ── 2. Odometry topic ──────────────────────────────────────
        # Same pose data, packaged as nav_msgs/Odometry for Nav2
        odom = Odometry()
        odom.header.stamp    = ros_stamp
        odom.header.frame_id = self.parent_frame  # "odom"
        odom.child_frame_id  = self.child_frame   # "base_link"

        # Pose — direct copy from gz
        odom.pose.pose.position.x    = pose.position.x
        odom.pose.pose.position.y    = pose.position.y
        odom.pose.pose.position.z    = pose.position.z
        odom.pose.pose.orientation.x = pose.orientation.x
        odom.pose.pose.orientation.y = pose.orientation.y
        odom.pose.pose.orientation.z = pose.orientation.z
        odom.pose.pose.orientation.w = pose.orientation.w

        # Twist — we don't have velocity from this gz topic so leave as zeros.
        # Nav2 velocity_smoother uses OPEN_LOOP feedback mode in your
        # nav2_params.yaml so it doesn't actually need real velocity here.

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = GzOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()