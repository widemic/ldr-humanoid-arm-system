#!/usr/bin/env python3
"""
Launch file for 3D perception system

Launches the perception node that detects geometric shapes from camera point clouds
and publishes them as MoveIt collision objects.

Usage:
    ros2 launch arm_perception perception.launch.py
    ros2 launch arm_perception perception.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo"
        ),
    ]

    # Get package directory
    arm_perception_dir = FindPackageShare("arm_perception")

    # Configuration file path
    config_path = PathJoinSubstitution([
        arm_perception_dir,
        "config",
        "perception.yaml"
    ])

    # Perception node
    perception_node = Node(
        package="arm_perception",
        executable="perception_node.py",
        name="perception_node",
        output="screen",
        parameters=[
            config_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            perception_node,
        ]
    )
