#!/usr/bin/env python3
"""
MoveIt client launch (laptop side).

Starts:
- RViz with MoveIt plugins for planning/execution requests to remote move_group.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            rviz,
        ]
    )
