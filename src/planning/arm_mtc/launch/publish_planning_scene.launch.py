#!/usr/bin/env python3
"""
Launch file for Planning Scene Publisher

Publishes collision objects (tables and cylinder) to the MoveIt planning scene
using configuration from pick_place_scene.yaml

Usage:
    ros2 launch arm_mtc publish_planning_scene.launch.py

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
        get_package_share_directory('arm_mtc'),
        'config',
        'pick_place_scene.yaml'
    )

    # Planning Scene Publisher Node
    publisher_node = Node(
        package='arm_mtc',
        executable='planning_scene_publisher.py',
        name='planning_scene_publisher',
        output='screen',
        parameters=[scene_config]
    )

    return LaunchDescription([publisher_node])
