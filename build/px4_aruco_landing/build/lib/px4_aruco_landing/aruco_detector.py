#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import cv2
import json
import math


def reprojection_error_px(rvec, tvec, corners_px, K, D, marker_length):
    """Mean reprojection error in pixels for the 4 marker corners."""
    s = marker_length / 2.0
    obj_pts = np.array(
        [[-s,  s, 0],
         [ s,  s, 0],
         [ s, -s, 0],
         [-s, -s, 0]],
        dtype=np.float32
    )

    img_proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)  # (4,1,2)
    img_proj = img_proj.reshape(4, 2)  # (4,2)

    corners_px = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)
    return float(np.mean(np.linalg.norm(img_proj - corners_px, axis=1)))


def quat_from_yaw(yaw_rad: float):
    """Yaw-only quaternion about +Z axis: returns (x,y,z,w)"""
    half = 0.5 * yaw_rad
    return 0.0, 0.0, float(math.sin(half)), float(math.cos(half))


class ArucoDetector(Node):
    """
    Subscribes:
      - image_topic (sensor_msgs/Image)
      - camera_info_topic (sensor_msgs/CameraInfo)

    Publishes:
      - marker_pose (geometry_msgs/PoseStamped)  # pose of selected marker in camera_optical_frame
      - aruco_detections_json (std_msgs/String)  # JSON for all detections in the frame

    NOTE:
      This version publishes a *yaw-only* orientation quaternion to avoid jitter from roll/pitch noise.
    """

    def __init__(self):
        super().__init__("aruco_detector")

        # Params
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_length_m", 0.5)  # IMPORTANT: match marker size
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("target_id", -1)  # -1 means accept any ID

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        self.target_id = int(self.get_parameter("target_id").value)

        dict_name = self.get_parameter("aruco_dict").value
        self.aruco_dict = self._make_dict(dict_name)

        # Detector params (compat with different OpenCV versions)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self._printed_encoding = False

        self.pose_pub = self.create_publisher(PoseStamped, "marker_pose", 10)
        self.json_pub = self.create_publisher(String, "aruco_detections_json", 10)

        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(f"Listening image: {self.image_topic}")
        self.get_logger().info(f"Listening camera_info: {self.camera_info_topic}")
        self.get_logger().info(f"Marker length (m): {self.marker_length}, target_id: {self.target_id}")

    def _make_dict(self, name: str):
        if not hasattr(cv2.aruco, name):
            self.get_logger().warn(f"Unknown aruco_dict '{name}', defaulting to DICT_4X4_50")
            name = "DICT_4X4_50"
        dict_id = getattr(cv2.aruco, name)
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dict_id)
        return cv2.aruco.Dictionary_get(dict_id)

    def on_camera_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if self.K[0, 0] <= 0 or self.K[1, 1] <= 0:
            self.get_logger().warn(f"Bad intrinsics fx/fy: {self.K[0,0]}, {self.K[1,1]}")

        if len(msg.d) > 0:
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.D = np.zeros((5, 1), dtype=np.float64)

    def on_image(self, msg: Image):
        if self.K is None or self.D is None:
            return

        if not self._printed_encoding:
            self.get_logger().info(
                f"Incoming image encoding: {msg.encoding}, step: {msg.step}, size: {msg.width}x{msg.height}"
            )
            self._printed_encoding = True

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if img.dtype != np.uint8:
            img = img.astype(np.uint8, copy=False)

        enc = msg.encoding.lower()

        if len(img.shape) == 2:
            gray = img
        else:
            img = np.ascontiguousarray(img)
            if "rgba" in enc:
                gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
            elif "bgra" in enc:
                gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
            elif "rgb" in enc:
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            else:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        gray = np.ascontiguousarray(gray)

        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten().astype(int)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.K, self.D
        )

        detections = []
        best_idx = None
        best_err = float("inf")

        for i, marker_id in enumerate(ids):
            corners_px = corners[i][0].astype(float)     # (4,2)
            rvec = rvecs[i, 0, :].astype(float)          # (3,)
            tvec = tvecs[i, 0, :].astype(float)          # (3,)

            err_px = reprojection_error_px(rvec, tvec, corners_px, self.K, self.D, self.marker_length)
            score = 1.0 / (1.0 + err_px)

            detections.append({
                "id": int(marker_id),
                "corners_px": corners_px.tolist(),
                "rvec": rvec.tolist(),
                "tvec": tvec.tolist(),
                "reproj_err_px": float(err_px),
                "score": float(score),
            })

            # selection logic
            if self.target_id != -1:
                if marker_id == self.target_id:
                    best_idx = i
                    best_err = err_px
            else:
                if err_px < best_err:
                    best_err = err_px
                    best_idx = i

        # Publish JSON for all detections
        payload = {
            "header": {
                "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
                "frame_id": str(msg.header.frame_id),
            },
            "detections": detections,
        }

        json_msg = String()
        json_msg.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        self.json_pub.publish(json_msg)

        if best_idx is None:
            return

        # Selected pose
        rvec_sel = rvecs[best_idx, 0, :].reshape(3, 1).astype(np.float64)
        tvec_sel = tvecs[best_idx, 0, :].reshape(3).astype(np.float64)

        # Convert rvec -> R -> yaw (camera frame) -> yaw-only quaternion
        R, _ = cv2.Rodrigues(rvec_sel)

        # Yaw about camera +Z (optical axis)
        yaw = float(math.atan2(R[1, 0], R[0, 0]))
        qx, qy, qz, qw = quat_from_yaw(yaw)

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "camera_optical_frame"

        # OpenCV camera optical frame:
        #   +X right, +Y down, +Z forward
        pose.pose.position.x = float(tvec_sel[0])
        pose.pose.position.y = float(tvec_sel[1])
        pose.pose.position.z = float(tvec_sel[2])

        # Publish yaw-only orientation (stable for yaw control)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()