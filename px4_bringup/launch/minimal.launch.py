#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir = os.path.expanduser("~/PX4-Autopilot")

    # ---- Launch args ----
    world = LaunchConfiguration("world")
    vehicle = LaunchConfiguration("vehicle")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="walls",
        description="Gazebo world name (without .sdf), must be discoverable via GZ_SIM_RESOURCE_PATH."
    )

    declare_vehicle = DeclareLaunchArgument(
        "vehicle",
        default_value="gz_x500_mono_cam_down",
        description="PX4 SITL vehicle target (e.g., gz_x500_mono_cam_down)."
    )

    # ---- Resource paths ----
    this_dir = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(this_dir, "..", ".."))

    custom_worlds_dir = os.path.join(repo_root, "px4_custom", "worlds")
    custom_models_dir = os.path.join(repo_root, "px4_custom", "models")

    px4_models_dir = os.path.join(px4_dir, "Tools", "simulation", "gz", "models")
    px4_worlds_dir = os.path.join(px4_dir, "Tools", "simulation", "gz", "worlds")
    px4_resources_dir = os.path.join(px4_dir, "Tools", "simulation", "gz", "resources")

    resource_paths = []
    for p in [px4_models_dir, px4_worlds_dir, px4_resources_dir, custom_worlds_dir, custom_models_dir]:
        if os.path.isdir(p):
            resource_paths.append(p)

    export_gz = 'export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH' + "".join([f":{p}" for p in resource_paths]) + '"; '

    # ---- IMPORTANT: pass bash -lc command as substitutions list ----
    px4_bash_cmd = [
        TextSubstitution(text=export_gz),
        TextSubstitution(text='cd "'), TextSubstitution(text=px4_dir), TextSubstitution(text='" && '),
        TextSubstitution(text="PX4_GZ_WORLD="), world,
        TextSubstitution(text=" make px4_sitl "),
        vehicle
    ]

    return LaunchDescription([
        declare_world,
        declare_vehicle,

        ExecuteProcess(
            cmd=["bash", "-lc", px4_bash_cmd],
            output="screen",
            name="px4_sitl"
        ),

        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-lc", "MicroXRCEAgent udp4 -p 8888"],
                    output="screen",
                    name="microxrce_agent"
                )
            ]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])