#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
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
        TextSubstitution(text="/model/mono_cam/link/camera_link/sensor/camera/image")
    ]
    camera_info_topic = [
        TextSubstitution(text="/world/"), world,
        TextSubstitution(text="/model/"), drone,
        TextSubstitution(text="/model/mono_cam/link/camera_link/sensor/camera/camera_info")
    ]

    # GZ -> ROS only (use '[')
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
        TextSubstitution(text="/model/lidar/link/link/sensor/lidar_2d_v2/scan")
    ]

    # GZ -> ROS only (use '[')
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

    # ---- Safety velocity filter ----
    safety_vel_filter = Node(
        package="px4_lidar",
        executable="safety_vel_filter",
        name="safety_vel_filter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            # keep defaults unless you want to override:
            # "cmd_in": "/cmd_vel_raw",
            # "cmd_out": "/cmd_vel_safe",
            # "scan_topic": "/scan_fixed",
        }],
    )

    # ---- Static TF: base_link -> lidar_link ----
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        output="screen",
        arguments=["0.1", "0", "0.26", "0", "0", "0", "base_link", "lidar_link"],
        parameters=[{"use_sim_time": True}],
    )

    # ---- Scan frame fix: make /scan_fixed with frame_id=lidar_link ----
    # IMPORTANT: don't hardcode the in_topic; build it from world+drone
    scan_fix = Node(
        package="px4_lidar",
        executable="scan_frame_fix",
        name="scan_frame_fix",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            # "in_topic": "/world/walls/model/x500_mono_cam_down_0/model/lidar/link/link/sensor/lidar_2d_v2/scan",
            "in_topic": scan_topic,
            "out_topic": "/scan_fixed",
            "frame_id": "lidar_link",
        }],
    )

    # ---- ArUco detector ----
    aruco = Node(
        package="px4_aruco_landing",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "marker_length_m": 0.5,
            # "target_id": -1,
        }],
    )

    # ---- Clock bridge ----
    # Prefer world-scoped Gazebo clock -> ROS /clock
    # NOTE: We hardcode the remap for the default world "walls".
    # If you launch with a different world name, update the remap accordingly.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/world/walls/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
        remappings=[("/world/walls/clock", "/clock")],
    )

    costmap_params = os.path.join(
        get_package_share_directory("px4_lidar"),
        "config",
        "local_costmap.yaml"
    )

    local_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        namespace="local_costmap",
        output="screen",
        parameters=[costmap_params, {"use_sim_time": True}],
        remappings=[("scan", "/scan_fixed")]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_costmap",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["local_costmap/costmap"],
            # optional but good:
            "bond_timeout": 0.0,
        }],
    )

    return LaunchDescription([
        declare_world,
        declare_drone,

        # Start clock + sensor bridges immediately
        clock_bridge,
        lidar_bridge,
        camera_bridge,

        # Start safety filter immediately (so cmd chain exists early)
        safety_vel_filter,

        lifecycle_manager,

        # Delay TF + scan_fix + aruco slightly (model may take a moment to appear)
        TimerAction(period=2.0, actions=[
            px4_odom_tf,
            lidar_static_tf,
            scan_fix,
            aruco,
            local_costmap
        ]),
    ])