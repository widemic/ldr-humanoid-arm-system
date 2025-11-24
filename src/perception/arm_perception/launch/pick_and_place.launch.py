#!/usr/bin/env python3
"""
Pick and Place Launch File

Launches the pick and place task with proper MoveIt configuration.

Usage:
    ros2 launch arm_perception pick_and_place.launch.py

Prerequisites:
    - Gazebo running: ros2 launch arm_control sim.launch.py
    - MoveIt running: ros2 launch arm_moveit_config demo.launch.py
    - Test environment: ros2 launch arm_perception test_environment.launch.py

Author: LDR Humanoid Arm System
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate launch description with MoveIt configs"""

    # Load COMPLETE MoveIt configuration (matching demo.launch.py)
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"])
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

    # Pick and place task node with FULL configuration
    pick_and_place_node = Node(
        package='arm_perception',
        executable='pick_and_place_cylinder.py',
        name='pick_and_place_task',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        pick_and_place_node
    ])
