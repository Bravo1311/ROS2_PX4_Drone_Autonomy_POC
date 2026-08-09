#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_policy = LaunchConfiguration("use_policy")
    weights_path = LaunchConfiguration("weights_path")
    steps_per_chunk = LaunchConfiguration("steps_per_chunk")

    declare_use_policy = DeclareLaunchArgument(
        "use_policy",
        default_value="false",
        description="If true, run the flow-matching policy node instead of PD."
    )
    declare_weights_path = DeclareLaunchArgument(
        "weights_path",
        default_value="",
        description="Path to trained policy .pt checkpoint (required if use_policy=true)."
    )
    declare_steps_per_chunk = DeclareLaunchArgument(
        "steps_per_chunk",
        default_value="3",
    )

    pd_node = Node(
        package="px4_aruco_landing",
        executable="autoland_twist_publisher",
        name="autoland_twist",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_policy),
    )

    policy_node = Node(
        package="flow_landing_policy",
        executable="policy_node",
        name="flow_landing_policy",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "weights_path": weights_path,
            "steps_per_chunk": steps_per_chunk,
        }],
        condition=IfCondition(use_policy),
    )

    return LaunchDescription([
        declare_use_policy,
        declare_weights_path,
        declare_steps_per_chunk,
        pd_node,
        policy_node,
    ])