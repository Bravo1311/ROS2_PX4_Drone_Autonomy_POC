#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("px4_bringup")

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_cfg = os.path.join(pkg, "rviz", "map_view.rviz")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": use_sim_time,
        }],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser(
                "~/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/maps/walls_map.yaml"
            ),
            description="Path to saved map yaml",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        map_server,
        lifecycle,
        # rviz,
    ])