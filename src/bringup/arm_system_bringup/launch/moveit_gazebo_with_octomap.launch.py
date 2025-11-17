#!/usr/bin/env python3
"""
Complete Gazebo + MoveIt + OctoMap + RViz Launch File

Launches the full system with proper OctoMap visualization in RViz.

Usage:
    ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
    ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with OctoMap visualization"
        ),
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="Path to Gazebo world file (empty = default lab.sdf)"
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error)"
        ),
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    log_level = LaunchConfiguration("log_level")

    # Package directories
    arm_system_bringup_dir = FindPackageShare("arm_system_bringup")

    # 1. Launch base system (Gazebo + MoveIt)
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                arm_system_bringup_dir,
                "launch",
                "moveit_gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "use_rviz": "false",  # We'll launch RViz separately with custom config
            "world": world,
            "log_level": log_level,
        }.items(),
    )

    # 2. Launch octomap_server (after controllers are ready)
    octomap_server = TimerAction(
        period=10.0,  # Wait for system to be fully initialized
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        arm_system_bringup_dir,
                        "launch",
                        "octomap_server.launch.py"
                    ])
                ]),
                launch_arguments={
                    "use_sim_time": "true",
                    "pointcloud_topic": "/camera/depth/points",
                    "frame_id": "base_fixture_link",
                    "base_frame_id": "base_fixture_link",
                    "use_color_octomap": "true",
                }.items(),
            )
        ]
    )

    # 2b. Launch colorizer node (after octomap_server is running)
    from launch.actions import ExecuteProcess
    import os
    colorizer_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
        'colorize_octomap_markers.py'
    )

    colorizer_node = TimerAction(
        period=11.0,  # Start 1 second after octomap_server
        actions=[
            ExecuteProcess(
                cmd=['python3', colorizer_script],
                output='screen',
                shell=False,
            )
        ]
    )

    # 3. Launch RViz with OctoMap visualization config
    from launch_ros.actions import Node
    from launch.conditions import IfCondition
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
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

    rviz_config = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "moveit_with_octomap.rviz"
    ])

    # Check if overlay workspace exists for native OctoMap displays
    import os
    overlay_ws = os.path.expanduser("~/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib")
    additional_env = {}
    if os.path.exists(overlay_ws):
        # Prepend overlay plugin path to LD_LIBRARY_PATH
        current_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
        additional_env = {
            "LD_LIBRARY_PATH": f"{overlay_ws}:{current_ld_path}"
        }

    rviz_node = TimerAction(
        period=12.0,  # Wait for octomap_server to start
        actions=[
            Node(
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
                    {"use_sim_time": True},
                ],
                additional_env=additional_env,
                condition=IfCondition(use_rviz),
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            base_system,
            octomap_server,
            colorizer_node,
            rviz_node,
        ]
    )
