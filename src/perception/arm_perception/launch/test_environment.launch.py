#!/usr/bin/env python3
"""
Test Environment Launch File

Launches test objects (table + cylinder) for gripper testing in RViz/MoveIt.

Usage:
    # Launch only test environment (requires MoveIt already running)
    ros2 launch arm_perception test_environment.launch.py

    # Or launch with MoveIt:
    # Terminal 1: ros2 launch arm_moveit_config demo.launch.py
    # Terminal 2: ros2 launch arm_perception test_environment.launch.py

Author: LDR Humanoid Arm System
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with test environment publisher"""

    # Test environment publisher node
    test_env_node = Node(
        package='arm_perception',
        executable='test_environment_publisher.py',
        name='test_environment_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True  # Use simulation time if Gazebo is running
        }]
    )

    return LaunchDescription([
        test_env_node
    ])
