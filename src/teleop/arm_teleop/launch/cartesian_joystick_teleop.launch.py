#!/usr/bin/env python3
"""Launch the Cartesian joystick teleop stack.

This launch file starts:
1. joy_node - Joystick driver
2. joint_teleop_node - Backend that converts JointState commands to trajectories
3. cartesian_joystick_teleop.py - Converts joystick XYZ to joint positions via MoveIt IK

Prerequisites:
- Full system running (Gazebo + controllers)
- MoveIt move_group running (provides /compute_ik service)
- Joystick connected
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launch configurations
    command_topic = LaunchConfiguration("command_topic")
    execution_time = LaunchConfiguration("execution_time")
    controller_profile = LaunchConfiguration("controller_profile")
    joy_topic = LaunchConfiguration("joy_topic")
    joy_device = LaunchConfiguration("joy_device")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat = LaunchConfiguration("joy_autorepeat")
    debug_mode = LaunchConfiguration("debug_mode")
    cartesian_scale = LaunchConfiguration("cartesian_scale")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "command_topic",
                default_value="teleop/joint_commands",
                description="JointState command topic shared between teleop clients and backend.",
            ),
            DeclareLaunchArgument(
                "execution_time",
                default_value="0.2",
                description="Seconds allowed for each joint trajectory (shorter = more responsive).",
            ),
            DeclareLaunchArgument(
                "controller_profile",
                default_value="dualsense",
                description="Joystick mapping preset to use (dualsense, xbox).",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="Topic published by joy_node and consumed by cartesian teleop.",
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
            DeclareLaunchArgument(
                "debug_mode",
                default_value="false",
                description="Enable debug output showing XYZ coordinates and IK status.",
            ),
            DeclareLaunchArgument(
                "cartesian_scale",
                default_value="0.1",
                description="Cartesian movement scale in m/s at full joystick deflection.",
            ),
            # Nodes
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
                executable="cartesian_joystick_teleop.py",
                name="cartesian_joystick_teleop",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "joy_topic": joy_topic,
                        "controller_profile": controller_profile,
                        "debug_mode": debug_mode,
                        "cartesian_scale": cartesian_scale,
                    }
                ],
            ),
        ]
    )
