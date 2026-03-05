#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration("world")
    drone = LaunchConfiguration("drone")
    mode = LaunchConfiguration("mode")  # mapping | localization
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params_file = LaunchConfiguration("slam_params_file")
    default_slam_params = os.path.join(
        get_package_share_directory("px4_lidar"),
        "config",
        "slam_toolbox.yaml",
    )

    # --- Perception / bridges (your existing stack) ---
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("px4_bringup"),
                "launch",
                "perception.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
            "drone": drone,
        }.items(),
    )

    # Build Gazebo scan topic (matches your perception.launch.py bridge)
    gz_scan_topic = [
        TextSubstitution(text="/world/"), world,
        TextSubstitution(text="/model/"), drone,
        TextSubstitution(text="/model/lidar/link/link/sensor/lidar_2d_v2/scan"),
    ]

    # --- Always-on TF + scan fix plumbing (needed for BOTH mapping and localization) ---

    # PX4 odom -> TF (odom -> base_link)
    px4_odom_tf = Node(
        package="px4_lidar",
        executable="px4_odom_tf",
        name="px4_odom_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # static TF base_link -> lidar_link
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        output="screen",
        arguments=[
            "0.10", "0.0", "0.26",  # xyz
            "0", "0", "0", "1",     # qx qy qz qw
            "base_link", "lidar_link",
            "--ros-args", "-p", "use_sim_time:=true",
        ],
    )

    # Rewrites scan frame_id + republishes to /scan_fixed
    # (Assumes your existing px4_lidar scan_frame_fix node does:
    #   subscribe: <gz_scan_topic>
    #   publish:   /scan_fixed
    #   set frame_id: lidar_link)
    scan_frame_fix = Node(
        package="px4_lidar",
        executable="scan_frame_fix",
        name="scan_frame_fix",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": gz_scan_topic},
            {"output_topic": "/scan_fixed"},
            {"frame_id": "lidar_link"},
        ],
    )

    # (Optional) your aruco node – keep it if you want markers available in both modes
    aruco_detector = Node(
        package="px4_lidar",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- Mapping: slam_toolbox builds map live ---
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    )

    # --- Localization: map_server + AMCL (YAML/PGM) ---
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "global_frame_id": "map",
            "tf_broadcast": True,
            "transform_tolerance": 0.5,
            "scan_topic": "/scan_fixed",
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="walls"),
            DeclareLaunchArgument("drone", default_value="x500_mono_cam_down_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("mode", default_value="mapping", description="mapping | localization"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="slam_toolbox params yaml (used in mapping)",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=os.path.join(
                    get_package_share_directory("px4_bringup"),
                    "maps",
                    "walls_map.yaml",
                ),
                description="Nav2 map yaml (yaml+pgm) used in localization mode",
            ),

            # Start bridges/sensors
            perception_launch,

            # Mapping or localization stacks
            slam_mapping,
            map_server,
            amcl,
            lifecycle_manager,
        ]
    )