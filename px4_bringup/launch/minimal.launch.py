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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from px4_bringup import sim_config


def generate_launch_description():
    # PX4_DIR is set in the Docker image; the fallback keeps the original host path.
    px4_dir = os.path.expanduser(os.environ.get('PX4_DIR', '/mnt/ubuntu-data/PX4-Autopilot'))

    # ---- Launch args ----
    world = LaunchConfiguration('world')
    vehicle = LaunchConfiguration('vehicle')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=sim_config.world_name(),
        description=(
            'Gazebo world name (without .sdf), must be discoverable via '
            'GZ_SIM_RESOURCE_PATH. Default: $PX4_GZ_WORLD, see px4_bringup/sim_config.py.'
        ),
    )

    declare_vehicle = DeclareLaunchArgument(
        'vehicle',
        default_value=sim_config.vehicle_target(),
        description='PX4 SITL vehicle target. Default: $PX4_VEHICLE, see px4_bringup/sim_config.py.',
    )

    # /clock for every use_sim_time node, whichever other launch files are running.
    sim_clock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px4_bringup'), 'launch', 'sim_clock.launch.py'
            )
        ),
        launch_arguments={'world': world}.items(),
    )

    # Dashboard link: port 2019 is what the dashboard expects, and the address keeps
    # rosbridge (no authentication) on this computer; the dashboard server proxies to it.
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        ),
        launch_arguments={'port': '2019', 'address': '127.0.0.1'}.items(),
    )

    # ---- Resource paths ----
    this_dir = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(this_dir, '..', '..'))

    custom_worlds_dir = os.path.join(repo_root, 'px4_custom', 'worlds')
    custom_models_dir = os.path.join(repo_root, 'px4_custom', 'models')

    px4_models_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    px4_worlds_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    px4_resources_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'resources')

    resource_paths = []
    for p in [
        px4_models_dir,
        px4_worlds_dir,
        px4_resources_dir,
        custom_worlds_dir,
        custom_models_dir,
    ]:
        if os.path.isdir(p):
            resource_paths.append(p)

    export_gz = (
        'export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH'
        + ''.join([f':{p}' for p in resource_paths])
        + '"; '
    )

    # ---- IMPORTANT: pass bash -lc command as substitutions list ----
    px4_bash_cmd = [
        TextSubstitution(text=export_gz),
        TextSubstitution(text='cd "'),
        TextSubstitution(text=px4_dir),
        TextSubstitution(text='" && '),
        TextSubstitution(text='PX4_GZ_WORLD='),
        world,
        TextSubstitution(text=' make px4_sitl '),
        vehicle,
    ]

    return LaunchDescription(
        [
            declare_world,
            declare_vehicle,
            rosbridge_launch,
            sim_clock,
            ExecuteProcess(cmd=['bash', '-lc', px4_bash_cmd], output='screen', name='px4_sitl'),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=['bash', '-lc', 'MicroXRCEAgent udp4 -p 8888'],
                        output='screen',
                        name='microxrce_agent',
                    )
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )
