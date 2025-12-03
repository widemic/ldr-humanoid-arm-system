#!/usr/bin/env python3
"""
Controller spawner for the humanoid robot.

This launch file spawns the controllers for the humanoid robot.
It assumes controller_manager and robot_state_publisher are already running
(via Gazebo plugin and spawn_robot.launch.py).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Joint state broadcaster spawner
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Leg controller spawner
    spawner_leg = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    # Leg controller spawner
    spawner_left_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    return LaunchDescription([
        spawner_jsb,
        spawner_leg,
        spawner_left_arm,
    ])
