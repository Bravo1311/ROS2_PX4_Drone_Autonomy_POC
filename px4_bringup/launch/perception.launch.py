#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration("world")
    drone = LaunchConfiguration("drone")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="walls",
        description="Gazebo world name (must match what PX4 started)."
    )
    declare_drone = DeclareLaunchArgument(
        "drone",
        default_value="x500_mono_cam_down_0",
        description="Gazebo model instance name."
    )

    # ---- Camera topics ----
    image_topic = [
        TextSubstitution(text="/world/"), world,
        TextSubstitution(text="/model/"), drone,
        TextSubstitution(text="/link/camera_link/sensor/camera/image")
    ]
    camera_info_topic = [
        TextSubstitution(text="/world/"), world,
        TextSubstitution(text="/model/"), drone,
        TextSubstitution(text="/link/camera_link/sensor/camera/camera_info")
    ]

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            image_topic + [TextSubstitution(text='@sensor_msgs/msg/Image@gz.msgs.Image')],
            camera_info_topic + [TextSubstitution(text='@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo')],
        ],
    )

    # ---- LiDAR topic ----
    scan_topic = [
        TextSubstitution(text="/world/"), world,
        TextSubstitution(text="/model/"), drone,
        TextSubstitution(text="/link/link/sensor/lidar_2d_v2/scan")
    ]

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        output="screen",
        arguments=[
            scan_topic + [TextSubstitution(text='@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan')],
        ],
    )

    # ---- TF bridge (THIS is what RViz needs) ----
    # NOTE: Gazebo side publishes transforms on these topics in many setups
    tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tf_bridge",
        output="screen",
        arguments=[
            TextSubstitution(text="/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"),
            TextSubstitution(text="/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"),
        ],
    )

    aruco = Node(
        package="px4_aruco_landing",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "marker_length_m": 0.5,
        }],
    )

    return LaunchDescription([
        declare_world,
        declare_drone,
        tf_bridge,
        lidar_bridge,
        camera_bridge,
        aruco,
    ])