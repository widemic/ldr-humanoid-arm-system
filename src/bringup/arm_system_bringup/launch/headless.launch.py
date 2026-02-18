#!/usr/bin/env python3
"""
Headless Launch File for Humanoid Arm System

This launch file brings up the complete system WITHOUT any GUI (no Gazebo GUI, no RViz) for:
- Headless servers and cloud environments
- Performance-critical applications
- Automated testing and CI/CD pipelines
- SSH sessions without X11 forwarding

Includes:
1. Gazebo Harmonic server only (gzserver - no GUI)
2. MoveIt motion planning server
3. All necessary controllers (no visualization)

Usage:
    ros2 launch arm_system_bringup headless.launch.py

    # Custom world file
    ros2 launch arm_system_bringup headless.launch.py world:=/path/to/world.sdf

    # With debug logging
    ros2 launch arm_system_bringup headless.launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    """Generate launch description for headless operation."""

    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_gazebo"), "worlds", "lab2.sdf"]
            ),
            description="Path to Gazebo world file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo",
        )
    )

    # Get launch configurations
    world_file = LaunchConfiguration("world")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get package paths
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_arm_gazebo = FindPackageShare("arm_gazebo")
    pkg_ros_gz_bridge = FindPackageShare("ros_gz_bridge")

    # Get install prefix to resolve package:// URIs
    install_dir = get_package_prefix("arm_description")
    install_dir_hand = get_package_prefix("hand_description")

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join([
            os.path.join(install_dir, "share"),
            os.path.join(install_dir_hand, "share"),
        ])
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,  # spawn_arm.launch.py publishes this
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 1. Launch Gazebo server only (no GUI) with world loaded
    # Using -s flag for server mode and -r to start running immediately
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [world_file, " -r -s"],  # -s for server mode (no GUI), -r for auto-run
        }.items(),
    )

    # 2. Clock bridge - essential for simulation time synchronization
    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ros_gz_bridge, "launch", "clock_bridge.launch"]
            )
        ),
        launch_arguments={
            "bridge_name": "gz_clock_bridge",
        }.items(),
    )

    # 3. Spawn the arm robot
    spawn_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_arm_gazebo, "launch", "spawn_arm.launch.py"]
            )
        ),
    )

    # 3. MoveGroup node - delay to ensure controllers are spawned and active
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

    # Delay MoveGroup start to ensure controllers are active
    delayed_move_group = TimerAction(
        period=10.0,  # Wait for controllers to be fully active
        actions=[move_group_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_resource_path,
            gazebo_server,
            clock_bridge,
            spawn_arm,
            delayed_move_group,
        ]
    )
