#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    description_pkg = FindPackageShare('arm_description')
    urdf_path = PathJoinSubstitution([description_pkg, 'urdf', 'arm.urdf.xacro'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    urdf_path, ' ',
                    'use_sim:=true'
                ]),
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
