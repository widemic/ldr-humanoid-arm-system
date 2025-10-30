#!/usr/bin/env python3
"""Simple launch file for arm controllers."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch arm controllers."""

    # Controller config
    controller_config = PathJoinSubstitution([
        FindPackageShare('arm_control'),
        'config',
        'controllers.yaml'
    ])

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    # Arm controller (delayed start)
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config
        ],
        output='screen'
    )

    # Delay arm controller to ensure joint_state_broadcaster is ready
    delayed_arm_controller = TimerAction(
        period=2.0,
        actions=[arm_controller]
    )

    return LaunchDescription([
        joint_state_broadcaster,
        delayed_arm_controller
    ])
