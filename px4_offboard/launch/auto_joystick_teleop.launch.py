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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'px4_offboard'

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', default_value='True', description='Use simulated time'
    )

    controller_node = Node(
        package=package_name,
        executable='px4_offboard_mux.py',
        name='px4_offboard_mux',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )

    joystick_teleop_node = Node(
        package=package_name,
        executable='teleop_joystick.py',
        name='teleop_joystick',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )

    nav2_relay_node = Node(
        package=package_name,
        executable='nav2_cmd_relay.py',
        name='nav2_cmd_relay',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            os.path.join(get_package_share_directory('px4_offboard'), 'config', 'joy_config.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    safety_vel_filter = Node(
        package='px4_lidar',
        executable='safety_vel_filter',
        name='safety_vel_filter',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                # keep defaults unless you want to override:
                # "cmd_in": "/cmd_vel_raw",
                # "cmd_out": "/cmd_vel_safe",
                # "scan_topic": "/scan_fixed",
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            controller_node,
            joy_node,
            joystick_teleop_node,
            nav2_relay_node,
            safety_vel_filter,
        ]
    )
