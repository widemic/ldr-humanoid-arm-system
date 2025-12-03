#!/usr/bin/env python3
"""
Launch file for MTC Pick and Place Demo

This launch file starts the MTC pick-and-place demo for the LDR humanoid arm.
It loads all necessary MoveIt configurations and parameters.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="Execute the task after planning (true/false)"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "max_solutions",
            default_value="10",
            description="Maximum number of solutions to find"
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute = LaunchConfiguration("execute")
    max_solutions = LaunchConfiguration("max_solutions")

    # Get package directory
    arm_mtc_dir = get_package_share_directory("arm_mtc")

    # MTC parameters file
    mtc_node_params_file = os.path.join(arm_mtc_dir, "config", "mtc_node_params.yaml")

    # Get moveit_config package directory for capabilities file
    arm_moveit_config_dir = get_package_share_directory("arm_moveit_config")
    move_group_capabilities_file = os.path.join(arm_moveit_config_dir, "config", "move_group_capabilities.yaml")

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

    # Move Group Node (required for planning scene and MTC)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # MTC Node
    mtc_demo_node = Node(
        package="arm_mtc",
        executable="mtc_node",
        name="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            mtc_node_params_file,
            {'use_sim_time': use_sim_time},
            {'execute': execute},
            {'max_solutions': max_solutions},
        ],
    )

    return LaunchDescription(declared_arguments + [
        move_group_node,
        mtc_demo_node,
    ])
