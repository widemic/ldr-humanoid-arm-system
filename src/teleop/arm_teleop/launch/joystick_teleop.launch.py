#!/usr/bin/env python3
"""Launch the joystick teleop stack (joy driver + teleop nodes)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    command_topic = LaunchConfiguration("command_topic")
    execution_time = LaunchConfiguration("execution_time")
    controller_profile = LaunchConfiguration("controller_profile")
    joy_topic = LaunchConfiguration("joy_topic")
    joy_device = LaunchConfiguration("joy_device")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat = LaunchConfiguration("joy_autorepeat")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command_topic",
                default_value="teleop/joint_commands",
                description="JointState command topic shared between teleop clients and backend.",
            ),
            DeclareLaunchArgument(
                "execution_time",
                default_value="1.0",
                description="Seconds allowed for each joint trajectory sent by joint_teleop_node.",
            ),
            DeclareLaunchArgument(
                "controller_profile",
                default_value="dualsense",
                description="Joystick mapping preset to use (e.g., dualsense, xbox).",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="Topic published by joy_node and consumed by joystick_teleop.py.",
            ),
            DeclareLaunchArgument(
                "joy_device",
                default_value="/dev/input/js0",
                description="Input device passed to joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_deadzone",
                default_value="0.05",
                description="Deadzone configured on joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_autorepeat",
                default_value="0.0",
                description="Autorepeat rate for joy_node button events.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": joy_device,
                        "deadzone": joy_deadzone,
                        "autorepeat_rate": joy_autorepeat,
                    }
                ],
                remappings=[("joy", joy_topic)],
            ),
            Node(
                package="arm_teleop",
                executable="joint_teleop_node",
                name="joint_teleop_node",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "execution_time_sec": execution_time,
                    }
                ],
            ),
            Node(
                package="arm_teleop",
                executable="joystick_teleop.py",
                name="arm_joystick_teleop",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "joy_topic": joy_topic,
                        "controller_profile": controller_profile,
                    }
                ],
            ),
        ]
    )
