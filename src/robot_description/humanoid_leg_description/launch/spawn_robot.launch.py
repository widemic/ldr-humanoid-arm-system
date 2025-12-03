#!/usr/bin/env python3
"""
Spawn the humanoid robot in an already running Gazebo simulation.

This launch file only spawns the robot, assuming Gazebo is already running.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation hardware interface'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for robot links and joints'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='X position to spawn the robot'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Y position to spawn the robot'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Z position to spawn the robot'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Roll angle to spawn the robot'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Pitch angle to spawn the robot'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Yaw angle to spawn the robot'
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')
    prefix = LaunchConfiguration('prefix')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Get package install directories
    install_dir = get_package_prefix('humanoid_leg_description')

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share')
    )

    # Paths
    urdf_file = PathJoinSubstitution([
        FindPackageShare('humanoid_leg_description'),
        'urdf',
        'robot.urdf.xacro'
    ])

    # Get URDF via xacro
    humanoid_leg_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file,
        ' use_sim:=',
        use_sim,
        ' prefix:=',
        prefix
    ])

    robot_description = {'robot_description': humanoid_leg_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kbot_humanoid',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # Create launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Add environment variable
    ld.add_action(gz_resource_path)

    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_robot)

    return ld
