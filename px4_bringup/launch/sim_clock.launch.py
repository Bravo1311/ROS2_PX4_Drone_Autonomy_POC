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

"""Bridge the Gazebo world clock to the ROS /clock topic, once the world is really running.

Why it waits: PX4's startup script decides whether Gazebo is already running with
``gz topic -l | grep "^/world/.*/clock"``. Gazebo lists a topic as soon as anything
*subscribes* to it, so a clock bridge started before PX4 makes PX4 believe Gazebo is up. PX4
then never starts Gazebo and waits for a world that does not exist. So the bridge starts only
after the world answers on its scene/info service (the same readiness test PX4 uses).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

from px4_bringup import sim_config


def generate_launch_description():
    world = LaunchConfiguration('world')

    def text(value):
        return TextSubstitution(text=value)

    # '[' = Gazebo -> ROS only (quoted so the shell does not treat it as a glob).
    # Every node with use_sim_time:=True needs /clock.
    wait_then_bridge = [
        text('until gz service -i --service /world/'), world,
        text('/scene/info 2>&1 | grep -q "Service providers"; do sleep 1; done; '),
        text("exec ros2 run ros_gz_bridge parameter_bridge '/world/"), world,
        text("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' "
             '--ros-args -r __node:=clock_bridge -r /world/'), world,
        text('/clock:=/clock'),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=sim_config.world_name(),
                description='Gazebo world name (must match what PX4 started).',
            ),
            ExecuteProcess(
                cmd=['bash', '-c', wait_then_bridge], output='screen', name='clock_bridge'
            ),
        ]
    )
