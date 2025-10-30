#!/usr/bin/env python3
"""
Simple all-in-one launch file for Gazebo simulation with arm control.

Launches:
1. Gazebo with the arm robot
2. Controllers (joint_state_broadcaster + arm_controller)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch complete simulation."""

    # Include Gazebo simulation (from arm_gazebo package)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_gazebo'),
                'launch',
                'arm_world.launch.py'
            ])
        ])
    )

    # Spawn arm robot
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_gazebo'),
                'launch',
                'spawn_arm.launch.py'
            ])
        ])
    )

    # Load controllers
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_control'),
                'launch',
                'control.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_launch,
        control_launch
    ])
