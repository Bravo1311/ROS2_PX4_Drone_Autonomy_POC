#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import numpy as np
import cv2


class ArucoDetector(Node):
    """
    Subscribes:
      - image_topic (sensor_msgs/Image)
      - camera_info_topic (sensor_msgs/CameraInfo)
    Publishes:
      - marker_pose (geometry_msgs/PoseStamped)  # position = tvec (meters) in OpenCV camera frame
    """

    def __init__(self):
        super().__init__("aruco_detector")

        # Params
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_length_m", 0.5)           # IMPORTANT: match aruco.sdf marker size
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("target_id", -1)                  # -1 means accept any ID

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        self.target_id = int(self.get_parameter("target_id").value)
        self._printed_encoding = False


        dict_name = self.get_parameter("aruco_dict").value
        self.aruco_dict = self._make_dict(dict_name)
        # Works on older OpenCV builds
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()


        self.bridge = CvBridge()

        self.K = None
        self.D = None

        self.pose_pub = self.create_publisher(PoseStamped, "marker_pose", 10)

        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(f"Listening image: {self.image_topic}")
        self.get_logger().info(f"Listening camera_info: {self.camera_info_topic}")
        self.get_logger().info(f"Marker length (m): {self.marker_length}, target_id: {self.target_id}")

    def _make_dict(self, name: str):
        # Common names: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, etc.
        if not hasattr(cv2.aruco, name):
            self.get_logger().warn(f"Unknown aruco_dict '{name}', defaulting to DICT_4X4_50")
            name = "DICT_4X4_50"
        dict_id = getattr(cv2.aruco, name)
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dict_id)
        # older OpenCV
        return cv2.aruco.Dictionary_get(dict_id)



    def on_camera_info(self, msg: CameraInfo):
        # Camera matrix K is 3x3 row-major in msg.k
        self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if self.K[0,0] <= 0 or self.K[1,1] <= 0:
            self.get_logger().warn(f"Bad intrinsics fx/fy: {self.K[0,0]}, {self.K[1,1]}")

        # Distortion may be empty in sim; handle gracefully
        if len(msg.d) > 0:
            D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            D = np.zeros((5, 1), dtype=np.float64)
        self.D = D

    def on_image(self, msg: Image):
        if self.K is None:
            return
        if not self._printed_encoding:
            self.get_logger().info(f"Incoming image encoding: {msg.encoding}, step: {msg.step}, size: {msg.width}x{msg.height}")
            self._printed_encoding = True

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if img.dtype != np.uint8:
            img = img.astype(np.uint8, copy=False)

        enc = msg.encoding.lower()

        if len(img.shape) == 2:
            gray = img
        else:
            img = np.ascontiguousarray(img)

            if "rgba" in enc or "bgra" in enc:
                # 4-channel
                if "rgba" in enc:
                    gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
                else:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)

            elif "rgb" in enc:
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

            else:
                # assume bgr if unknown
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        gray = np.ascontiguousarray(gray)



        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )


        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()

        # Choose target ID if requested; else first marker
        idx = 0
        if self.target_id != -1:
            matches = np.where(ids == self.target_id)[0]
            if len(matches) == 0:
                return
            idx = int(matches[0])

        # estimatePoseSingleMarkers returns rvecs/tvecs for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.K, self.D
        )

        tvec = tvecs[idx].reshape((3,))
        rvec = rvecs[idx].reshape((3,))

        # Publish PoseStamped: position = tvec (meters) in OpenCV camera frame
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "camera_optical_frame"  # conceptual; you're in OpenCV optical frame

        out.pose.position.x = float(tvec[0])  # +X right
        out.pose.position.y = float(tvec[1])  # +Y down
        out.pose.position.z = float(tvec[2])  # +Z forward (for down cam, forward ≈ down)

        # Orientation is optional for landing; leave quaternion as identity for now
        out.pose.orientation.w = 1.0

        self.pose_pub.publish(out)


def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
