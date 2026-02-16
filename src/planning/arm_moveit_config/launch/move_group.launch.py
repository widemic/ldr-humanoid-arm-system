#!/usr/bin/env python3
"""
MoveIt server launch (Jetson side).

Starts:
- robot_state_publisher
- move_group (MoveIt planning/execution server)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {
                "octomap_frame": "base_link",
                "octomap_resolution": 0.05,
                "max_range": 5.0,
            },
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            move_group,
        ]
    )
