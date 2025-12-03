#!/usr/bin/env python3
"""
MTC RViz - Standalone RViz with MTC visualization

Keep this running in a separate terminal. Use mtc_run.launch.py to execute tasks.

Usage:
    # Terminal 1: Start the full system
    ros2 launch arm_system_bringup full_system.launch.py
    
    # Terminal 2: Start RViz with MTC panels (keep running)
    ros2 launch arm_mtc mtc_rviz.launch.py
    
    # Terminal 3: Run MTC tasks (can run multiple times)
    ros2 launch arm_mtc mtc_run.launch.py
"""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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

    # MoveIt move_group node with MTC execution capability
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
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
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
