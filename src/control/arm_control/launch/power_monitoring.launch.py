#!/usr/bin/env python3
"""
Launch file for power monitoring system.

Usage:
    # With CSV logging
    ros2 launch arm_control power_monitoring.launch.py log_to_file:=true

    # Custom log path
    ros2 launch arm_control power_monitoring.launch.py \
        log_to_file:=true \
        log_file_path:=/path/to/power_log.csv

    # Custom update rate
    ros2 launch arm_control power_monitoring.launch.py update_rate:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for power monitoring."""

    # Get package paths
    arm_control_share = get_package_share_directory('arm_control')
    actuator_specs_path = os.path.join(arm_control_share, 'config', 'actuator_specs.yaml')

    # Declare launch arguments
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )

    log_to_file_arg = DeclareLaunchArgument(
        'log_to_file',
        default_value='false',
        description='Enable CSV logging'
    )

    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value='/tmp/power_log.csv',
        description='Path to CSV log file'
    )

    # Power monitor node
    power_monitor_node = Node(
        package='arm_control',
        executable='power_monitor_node.py',
        name='power_monitor_node',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'actuator_specs_path': actuator_specs_path,
            'log_to_file': LaunchConfiguration('log_to_file'),
            'log_file_path': LaunchConfiguration('log_file_path'),
        }]
    )

    return LaunchDescription([
        update_rate_arg,
        log_to_file_arg,
        log_file_path_arg,
        power_monitor_node,
    ])
