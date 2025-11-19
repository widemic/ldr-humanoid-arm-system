#!/usr/bin/env python3
"""
Launch file for 3D perception system with real-time collision detection

Launches the perception system with:
- OctoMap server for 3D environment mapping
- Perception node for object detection
- Planning scene monitor for real-time collision updates to MoveIt

Usage:
    ros2 launch arm_perception perception.launch.py
    ros2 launch arm_perception perception.launch.py use_sim_time:=true
    ros2 launch arm_perception perception.launch.py enable_octomap:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo"
        ),
        DeclareLaunchArgument(
            "enable_octomap",
            default_value="true",
            description="Enable OctoMap server for real-time collision detection"
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/camera/depth/points",
            description="Point cloud topic from depth camera"
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="base_link",
            description="Reference frame for OctoMap"
        ),
        DeclareLaunchArgument(
            "enable_object_tracking",
            default_value="true",
            description="Enable dynamic object tracking for moving objects"
        ),
    ]

    # Get launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_octomap = LaunchConfiguration("enable_octomap")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    frame_id = LaunchConfiguration("frame_id")
    enable_object_tracking = LaunchConfiguration("enable_object_tracking")

    # Get package directories
    arm_perception_dir = FindPackageShare("arm_perception")
    arm_system_bringup_dir = FindPackageShare("arm_system_bringup")

    # Configuration file path
    perception_config = PathJoinSubstitution([
        arm_perception_dir,
        "config",
        "perception.yaml"
    ])

    # 1. Perception node for object detection
    perception_node = Node(
        package="arm_perception",
        executable="perception_node.py",
        name="perception_node",
        output="screen",
        parameters=[
            perception_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2. OctoMap server for real-time environment mapping
    octomap_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                arm_system_bringup_dir,
                "launch",
                "octomap_server.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "pointcloud_topic": pointcloud_topic,
            "frame_id": frame_id,
            "base_frame_id": frame_id,
            "use_color_octomap": "true",
        }.items(),
        condition=IfCondition(enable_octomap),
    )

    # 3. Planning scene updater node - publishes OctoMap to MoveIt planning scene
    planning_scene_updater = Node(
        package="arm_perception",
        executable="planning_scene_updater.py",
        name="planning_scene_updater",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "update_rate": 5.0,  # Update planning scene at 5 Hz
                "octomap_frame": frame_id,
            }
        ],
        condition=IfCondition(enable_octomap),
    )

    # 4. Dynamic object tracker - tracks moving objects with persistent IDs
    # OPTIMIZED for complex objects like humans
    dynamic_object_tracker = Node(
        package="arm_perception",
        executable="dynamic_object_tracker.py",
        name="dynamic_object_tracker",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "min_cluster_size": 100,          # Increased to filter small noise
                "max_cluster_size": 10000,        # Increased for large objects (humans)
                "cluster_tolerance": 0.15,        # Increased for complex shapes
                "max_tracking_distance": 0.5,     # Increased for faster movements
                "object_timeout": 2.0,            # Remove objects not seen for 2s
                "min_object_height": 0.05,        # 5cm - filters ground plane better
                "reference_frame": frame_id,
            }
        ],
        condition=IfCondition(enable_object_tracking),
    )

    return LaunchDescription(
        declared_arguments + [
            perception_node,
            octomap_server,
            TimerAction(
                period=2.0,  # Wait 2s for octomap_server to initialize
                actions=[planning_scene_updater]
            ),
            dynamic_object_tracker,  # Start immediately (doesn't depend on octomap)
        ]
    )
