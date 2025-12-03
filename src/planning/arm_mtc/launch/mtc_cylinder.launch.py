#!/usr/bin/env python3
"""
MTC Pick Place Cylinder - Standalone Launch

This launch file assumes full_system.launch.py is already running.
It only starts:
  1. MoveIt move_group (if not already running)
  2. RViz with MTC visualization
  3. The MTC pick_place_cylinder node

Usage:
    # First terminal: Start the full system
    ros2 launch arm_system_bringup full_system.launch.py
    
    # Second terminal: Run this MTC demo
    ros2 launch arm_mtc mtc_cylinder.launch.py
    
    # Or without RViz (if you already have one open):
    ros2 launch arm_mtc mtc_cylinder.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with MTC visualization'
    )
    
    execute_arg = DeclareLaunchArgument(
        'execute',
        default_value='true',
        description='Execute the planned trajectory on the robot'
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # RViz with MTC config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arm_mtc'),
        'config',
        'mtc.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mtc',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # MTC pick place cylinder node - wait 3 seconds for move_group to initialize
    mtc_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='arm_mtc',
                executable='mtc_pick_place_cylinder',
                name='mtc_pick_place_cylinder',
                output='screen',
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': True},
                    {'execute': LaunchConfiguration('execute')},
                ],
            )
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        execute_arg,
        move_group_node,
        rviz_node,
        mtc_node,
    ])
