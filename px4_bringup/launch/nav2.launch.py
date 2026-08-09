#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav2_params_file = os.path.join(
        get_package_share_directory('px4_bringup'),
        'config',
        'nav2_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ── Global costmap ───────────────────────────────────────────
    # Full map-sized grid. Used by planner to find obstacle-free
    # paths. Has static layer (SLAM map) + obstacle layer (lidar).
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan_fixed')]
    )

    # ── Local costmap ────────────────────────────────────────────
    # Small rolling window around the drone.
    # Used by DWB to score trajectories in real time.
    # Only live lidar — no static layer.
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan_fixed')]
    )

    # ── Controller server ────────────────────────────────────────
    # Runs DWB local planner. Takes global path + local costmap,
    # outputs /cmd_vel_nav_raw at 10Hz.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel_nav_raw')]
    )

    # ── Smoother server ──────────────────────────────────────────
    # Smooths the global path geometry before DWB follows it.
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Planner server ───────────────────────────────────────────
    # Computes global path from current pose to goal using
    # the static map. Runs NavFn (A*).
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Behavior server ──────────────────────────────────────────
    # Recovery behaviours — spin, backup, wait.
    # Triggered automatically by bt_navigator when stuck.
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── BT Navigator ─────────────────────────────────────────────
    # Orchestrates the whole navigation action.
    # Receives /goal_pose from RViz, calls planner + controller,
    # monitors progress, triggers recovery if stuck.
    # Exposes /navigate_to_pose action server.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Waypoint follower ────────────────────────────────────────
    # Executes sequences of waypoints.
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    # ── Velocity smoother ────────────────────────────────────────
    # Smooths /cmd_vel_nav_raw from DWB into /cmd_vel_nav.
    # Limits acceleration so commands aren't jerky.
    # OPEN_LOOP mode — doesn't need real velocity from /odom.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',          '/cmd_vel_nav_raw'),  # input from DWB
            ('cmd_vel_smoothed', '/cmd_vel_nav'),       # output to nav2_cmd_relay
        ]
    )

    # ── Lifecycle manager ────────────────────────────────────────
    # Manages startup and shutdown order of all Nav2 nodes.
    # Nav2 nodes are lifecycle nodes — they must be configured
    # and activated in the right order before they do anything.
    # autostart: true does this automatically on launch.
    # bond_timeout increased to handle slow map loading.
    # attempt_respawn_reconnection: retry if a node is slow.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time':                   use_sim_time,
            'autostart':                      True,
            'bond_timeout':                   10.0,
            'attempt_respawn_reconnection':   True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,

        # Wait 5 seconds before starting anything.
        # This gives SLAM Toolbox time to publish /map with
        # TRANSIENT_LOCAL durability so the global costmap's
        # static layer receives it before trying to configure.
        # Without this delay the planner_server times out
        # during lifecycle configure and bt_navigator never activates.
        TimerAction(period=5.0, actions=[
            # global_costmap,
            # local_costmap,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ])
    ])