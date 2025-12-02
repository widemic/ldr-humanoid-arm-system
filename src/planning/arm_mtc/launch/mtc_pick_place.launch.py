#!/usr/bin/env python3
"""
Launch file for MTC Pick and Place (C++ implementation)

This launch file starts the MTC pick-and-place task using the C++ implementation.
It requires move_group to be running with proper MoveIt configuration.

Usage:
    ros2 launch arm_mtc mtc_pick_place.launch.py

Prerequisites:
    - Gazebo + controllers: ros2 launch arm_system_bringup full_system.launch.py
    - MoveIt move_group: Must be running (included in full_system or demo.launch.py)
    - Planning scene: ros2 run arm_mtc planning_scene_publisher.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true"
        )
    )

    # Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Load scene configuration YAML
    scene_config = os.path.join(
        get_package_share_directory('arm_mtc'),
        'config',
        'pick_place_scene.yaml'
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # MTC Pick and Place Node
    mtc_node = Node(
        package="arm_mtc",
        executable="pick_place_task",
        name="pick_place_task",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            scene_config,  # Load scene configuration from YAML
            {
                "use_sim_time": use_sim_time,
                # MTC specific parameters
                "execute": True,  # Execute task after planning
            },
        ],
    )

    # Delay node start to ensure move_group is ready
    delayed_node = TimerAction(
        period=3.0,
        actions=[mtc_node]
    )

    return LaunchDescription(declared_arguments + [
        delayed_node
    ])
