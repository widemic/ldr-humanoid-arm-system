#!/usr/bin/env python3
"""
Launch Gazebo simulation with the humanoid robot.

This launch file:
1. Starts Gazebo Harmonic with a world file
2. Spawns the humanoid robot from humanoid_leg_description
3. Sets up the ROS-Gazebo bridge for topics
4. Configures robot_state_publisher
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='World file name (in worlds/ directory)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
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

    # Initialize Arguments
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    use_sim = LaunchConfiguration('use_sim')
    prefix = LaunchConfiguration('prefix')

    # Get package install directories
    install_dir = get_package_prefix('humanoid_leg_description')

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share')
    )

    # Set GZ_SIM_SYSTEM_PLUGIN_PATH to find gz_ros2_control plugin
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # Paths
    world_file = PathJoinSubstitution([
        FindPackageShare('humanoid_leg_description'),
        'worlds',
        world
    ])

    urdf_file = PathJoinSubstitution([
        FindPackageShare('humanoid_leg_description'),
        'urdf',
        'robot.urdf.xacro'
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare('humanoid_leg_description'),
        'config',
        'controllers.yaml'
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

    # Gazebo simulation using ros_gz_sim launch file
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([
                '-r ',  # Run on start
                world_file
            ]),
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kbot_humanoid',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Controller spawner nodes using timers
    # Wait for Gazebo's controller_manager to be ready
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to initialize
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--params-file",
                controllers_file,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen",
        )]
    )

    leg_controller_spawner = TimerAction(
        period=6.0,  # Wait 6 seconds (after joint_state_broadcaster)
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "leg_controller",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--params-file",
                controllers_file,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen",
        )]
    )


    left_arm_controller_spawner = TimerAction(
        period=6.0,  # Wait 6 seconds (after joint_state_broadcaster)
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "left_arm_controller",
                "--controller-manager",
                "/controller_manager",
                "--ros-args",
                "--params-file",
                controllers_file,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen",
        )]
    )
    # Create launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Add environment variables
    ld.add_action(gz_resource_path)
    ld.add_action(gz_plugin_path)

    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_sim)
    ld.add_action(spawn_robot)
    ld.add_action(clock_bridge)

    # Add controller spawners with delays
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(leg_controller_spawner)
    ld.add_action(left_arm_controller_spawner)

    return ld
