#!/usr/bin/env python3
"""
Launch file for Test Environment Publisher

Spawns test tables and cylinder into the MoveIt planning scene
using configuration from pick_place_scene.yaml

Usage:
    ros2 launch arm_perception spawn_test_environment.launch.py

Prerequisites:
    - MoveIt move_group must be running
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Load scene configuration YAML
    scene_config = os.path.join(
        get_package_share_directory('arm_perception'),
        'config',
        'pick_place_scene.yaml'
    )

    # Test Environment Publisher Node
    spawner_node = Node(
        package='arm_perception',
        executable='test_environment_publisher.py',
        name='test_environment_publisher',
        output='screen',
        parameters=[scene_config]
    )

    return LaunchDescription([spawner_node])
