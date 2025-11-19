#!/usr/bin/env python3
"""Launch file for the arm keyboard teleoperation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    step = LaunchConfiguration("step")
    execution_time = LaunchConfiguration("execution_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "step",
                default_value="0.05",
                description="Joint increment (rad) applied per key press.",
            ),
            DeclareLaunchArgument(
                "execution_time",
                default_value="1.0",
                description="Seconds allowed for each joint trajectory.",
            ),
            Node(
                package="arm_teleop",
                executable="joint_teleop_node",
                name="joint_teleop_node",
                output="screen",
                parameters=[
                    {
                        "command_topic": "teleop/joint_commands",
                        "execution_time_sec": execution_time,
                    }
                ],
            ),
            Node(
                package="arm_teleop",
                executable="keyboard_teleop.py",
                name="arm_keyboard_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "step": step,
                        "command_topic": "teleop/joint_commands",
                    }
                ],
            ),
        ]
    )
