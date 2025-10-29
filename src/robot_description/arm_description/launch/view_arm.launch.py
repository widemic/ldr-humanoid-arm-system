#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('arm_description')

    # Path to URDF file
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'arm.urdf.xacro'
    ])

    # Path to RViz config (optional)
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'view_arm.rviz'
    ])

    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (gazebo) or real robot configuration for URDF variant'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock from Gazebo (true) or system clock (false)'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui'
    )

    use_rviz_config_arg = DeclareLaunchArgument(
        'use_rviz_config',
        default_value='false',
        description='Use saved RViz configuration file'
    )

    # Launch configuration variables
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz_config = LaunchConfiguration('use_rviz_config')

    # Robot State Publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file, ' use_sim:=', use_sim])
        }]
    )

    # Joint State Publisher (non-GUI fallback)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(gui)
    )

    # Joint State Publisher GUI - interactive joint control sliders
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui)
    )

    # RViz with config file
    rviz_with_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz_config)
    )

    # RViz without config file (default)
    rviz_default = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_rviz_config)
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_arg,
        gui_arg,
        use_rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_default,
        rviz_with_config
    ])
