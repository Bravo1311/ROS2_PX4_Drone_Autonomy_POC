#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    # ---- LiDAR topic (bridged, unchanged) ----
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

    # ---- PX4 odom -> TF (odom -> base_link) ----
    px4_odom_tf = Node(
        package="px4_lidar",
        executable="px4_odom_tf",
        name="px4_odom_tf",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    
    safety_vel_filter = Node(
        package="px4_lidar",
        executable="safety_vel_filter",
        name="safety_vel_filter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ---- Static TF: base_link -> lidar_link ----
    # (Adjust xyz/rpy later if lidar is offset/rotated)
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        output="screen",
        arguments=["0.1", "0", "0.26", "0", "0", "0", "base_link", "lidar_link"],
        parameters=[{"use_sim_time": True}],
    )

    # ---- Scan frame fix: republish scan with frame_id=lidar_link on /scan_fixed ----
    # NOTE: parameters are hardcoded for your current world+drone defaults.
    scan_fix = Node(
        package="px4_lidar",
        executable="scan_frame_fix",
        name="scan_frame_fix",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "in_topic": "/world/walls/model/x500_mono_cam_down_0/link/link/sensor/lidar_2d_v2/scan",
            "out_topic": "/scan_fixed",
            "frame_id": "lidar_link",
        }],
    )

    aruco = Node(
        package="px4_aruco_landing",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "image_topic": image_topic,
            "use_sim_time": True,
            "camera_info_topic": camera_info_topic,
            "marker_length_m": 0.5,
        }],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
    )

    return LaunchDescription([
        declare_world,
        declare_drone,
        clock_bridge,
        lidar_bridge,
        camera_bridge,
        safety_vel_filter,
        TimerAction(period=2.0, actions=[
            px4_odom_tf,
            lidar_static_tf,
            scan_fix,
            aruco
        ]),
    ])