#!/usr/bin/env python3
"""
PX4 Minimal Launch File
Launches PX4 SITL and MicroXRCE Agent
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')

    # Pick the world name you want (must exist under Tools/simulation/gz/worlds/)
    # gz_world = "aruco-moving-platform"
    gz_world = "aruco"

    return LaunchDescription([
        # 1) Launch PX4 SITL (gz sim) in a new terminal
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL',
                '--',
                'bash', '-lc',
                # IMPORTANT: pass PX4_GZ_WORLD as a make argument (more reliable than env prefix)
                f'cd "{px4_dir}" && make px4_sitl gz_x500_mono_cam_down PX4_GZ_WORLD={gz_world}; exec bash'
            ],
            output='screen',
            name='px4_sitl'
        ),

        # 2) Launch MicroXRCE Agent after a short delay
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-lc',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),

        # (Optional) QGroundControl after another delay
        # TimerAction(
        #     period=8.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 'bash', '-lc',
        #                 f'chmod +x "{qgc_path}" && "{qgc_path}"'
        #             ],
        #             output='screen',
        #             name='qgroundcontrol'
        #         )
        #     ]
        # ),
    ])
