#!/usr/bin/env python3
"""
Simple pick demo launch file - loads MoveIt configs for the Python script
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # Simple pick demo node
    simple_pick_node = Node(
        package="arm_mtc",
        executable="simple_pick.py",
        name="simple_pick_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        simple_pick_node,
    ])
