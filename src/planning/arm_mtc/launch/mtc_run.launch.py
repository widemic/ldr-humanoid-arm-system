#!/usr/bin/env python3
"""
MTC Run - Execute MTC pick and place task only

This launch file ONLY runs the MTC node. Assumes:
  1. full_system.launch.py is running
  2. mtc_rviz.launch.py is running (move_group + RViz)

Usage:
    # Terminal 1: Start the full system
    ros2 launch arm_system_bringup full_system.launch.py
    
    # Terminal 2: Start RViz with MTC panels (keep running)
    ros2 launch arm_mtc mtc_rviz.launch.py
    
    # Terminal 3: Run MTC tasks (can run multiple times)
    ros2 launch arm_mtc mtc_run.launch.py
    
    # With custom object parameters:
    ros2 launch arm_mtc mtc_run.launch.py object_id:=my_cylinder pick_x:=-0.2 pick_y:=0.5 pick_z:=1.2
    
    # Plan only (don't execute on robot):
    ros2 launch arm_mtc mtc_run.launch.py execute:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('execute', default_value='true',
                              description='Execute the planned trajectory on the robot'),
        DeclareLaunchArgument('object_id', default_value='target_cylinder',
                              description='ID of the object to pick'),
        DeclareLaunchArgument('object_radius', default_value='0.025',
                              description='Radius of the cylinder in meters'),
        DeclareLaunchArgument('object_height', default_value='0.15',
                              description='Height of the cylinder in meters'),
        DeclareLaunchArgument('pick_x', default_value='-0.2',
                              description='Pick X position'),
        DeclareLaunchArgument('pick_y', default_value='0.5',
                              description='Pick Y position'),
        DeclareLaunchArgument('pick_z', default_value='1.2',
                              description='Pick Z position'),
        DeclareLaunchArgument('place_x', default_value='-0.2',
                              description='Place X position'),
        DeclareLaunchArgument('place_y', default_value='-0.5',
                              description='Place Y position'),
        DeclareLaunchArgument('place_z', default_value='1.2',
                              description='Place Z position'),
        DeclareLaunchArgument('spawn_object', default_value='true',
                              description='Whether to spawn the object in planning scene'),
        DeclareLaunchArgument('spawn_table', default_value='true',
                              description='Whether to spawn the table in planning scene'),
    ]

    # Build MoveIt configuration (needed for robot model)
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

    # MTC pick place cylinder node
    mtc_node = Node(
        package='arm_mtc',
        executable='mtc_pick_place_cylinder',
        name='mtc_pick_place_cylinder',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'execute': LaunchConfiguration('execute')},
            {'object_id': LaunchConfiguration('object_id')},
            {'object_radius': LaunchConfiguration('object_radius')},
            {'object_height': LaunchConfiguration('object_height')},
            {'pick_x': LaunchConfiguration('pick_x')},
            {'pick_y': LaunchConfiguration('pick_y')},
            {'pick_z': LaunchConfiguration('pick_z')},
            {'place_x': LaunchConfiguration('place_x')},
            {'place_y': LaunchConfiguration('place_y')},
            {'place_z': LaunchConfiguration('place_z')},
            {'spawn_object': LaunchConfiguration('spawn_object')},
            {'spawn_table': LaunchConfiguration('spawn_table')},
        ],
    )

    return LaunchDescription(declared_arguments + [mtc_node])
