#!/usr/bin/env python3
"""
Launch only MoveIt + OctoMap + RViz (no Gazebo)

This assumes Gazebo is already running with the robot spawned.
Use this after launching Gazebo separately.

Usage:
    # Terminal 1: Launch Gazebo first
    ros2 launch arm_gazebo headless_sim.launch.py
    # or
    ros2 launch arm_system_bringup gazebo_only.launch.py

    # Terminal 2: Launch MoveIt + OctoMap
    ros2 launch arm_system_bringup moveit_octomap_only.launch.py
    ros2 launch arm_system_bringup moveit_octomap_only.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with OctoMap visualization"
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error)"
        ),
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,  # Gazebo already publishes
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"publish_monitored_planning_scene": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # OctoMap server node
    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"resolution": 0.05},
            {"frame_id": "base_link"},
            {"sensor_model/max_range": 5.0},
            {"sensor_model/min_range": 0.1},
            {"filter_ground": False},
            {"base_frame_id": "base_link"},
            {"height_map": False},
            {"colored_map": True},
            {"color/r": 0.0},
            {"color/g": 0.0},
            {"color/b": 1.0},
            {"color/a": 1.0},
        ],
        remappings=[
            ("cloud_in", "/camera/depth/points"),
        ],
    )

    # Delay OctoMap server to ensure MoveIt is ready
    delayed_octomap = TimerAction(
        period=2.0,
        actions=[octomap_server_node],
    )

    # RViz configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "moveit_with_octomap.rviz"
    ])

    # Check if overlay workspace exists for native OctoMap displays
    overlay_ws = os.path.expanduser("~/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib")
    additional_env = {}
    if os.path.exists(overlay_ws):
        additional_env = {
            "LD_LIBRARY_PATH": f"{overlay_ws}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        }

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        additional_env=additional_env,
        condition=IfCondition(use_rviz),
    )

    # Delay RViz to ensure everything is ready
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    return LaunchDescription(
        declared_arguments + [
            move_group_node,
            delayed_octomap,
            delayed_rviz,
        ]
    )
