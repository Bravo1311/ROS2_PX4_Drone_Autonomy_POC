#!/usr/bin/env python3

# Copyright 2026 Kartik Agrawal
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    world        = LaunchConfiguration('world')
    drone        = LaunchConfiguration('drone')
    mode         = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_slam_params = os.path.join(
        get_package_share_directory('px4_lidar'),
        'config',
        'slam_toolbox.yaml',
    )

    # ── Perception stack ─────────────────────────────────────────
    # Starts: clock bridge, lidar bridge, camera bridge, odom bridge,
    #         gz_odom_tf, lidar_static_tf, scan_frame_fix, aruco,
    #         safety_vel_filter.
    # We do NOT duplicate any of those here.
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px4_bringup'),
                'launch',
                'perception.launch.py',
            )
        ),
        launch_arguments={
            'world': world,
            'drone': drone,
        }.items(),
    )

    # ── SLAM Toolbox — mapping mode ──────────────────────────────
    # Builds a map live from /scan_fixed.
    # Publishes map → odom TF and /map topic.
    # Run this first to build and save your map.
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time':     use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'mapping'"])
        ),
    )

    # ── SLAM Toolbox — localization mode ─────────────────────────
    # Localizes on a previously saved map.
    # Still publishes map → odom TF and /map topic.
    # Use this when you want Nav2 goal navigation on a known map.
    # Launch with:
    #   ros2 launch px4_bringup slam.launch.py mode:=localization
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'localization_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time':     use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world',        default_value='walls'),
        DeclareLaunchArgument('drone',        default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='mapping | localization'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='SLAM Toolbox params yaml',
        ),

        perception_launch,
        slam_mapping,
        slam_localization,
    ])