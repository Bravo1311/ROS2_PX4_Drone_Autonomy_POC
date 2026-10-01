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


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # --- Configuration ---
    package_name = 'px4_offboard'  # Change this to your package name
    ros_distro = 'humble'         # Change this to your ROS 2 distro
    # ---------------------

    # Path to your ROS 2 setup file
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'

    # Launch the main offboard controller node
    controller_node = Node(
        package=package_name,
        executable='velocity_offboard_control.py',
        name='velocity_offboard_control',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )
    
    # Launch the custom teleop keyboard node in a new gnome-terminal
    teleop_node_process = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            # Command to run in the new terminal:
            f'source {ros_setup_path} && '
            f'echo "Terminal for Keyboard Teleop (Press Space to Arm)" && '
            f'ros2 run {package_name} teleop_keyboard.py; '
            'exec bash' # Keeps the terminal open after the script exits
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        controller_node,
        teleop_node_process
    ])