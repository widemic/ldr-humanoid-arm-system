#!/usr/bin/env python3
"""
Complete Gazebo + MoveIt + RViz Launch File for Humanoid Arm System

This launch file brings up:
1. Gazebo Harmonic simulation with the robot spawned
2. MoveIt motion planning server
3. RViz with MoveIt Motion Planning plugin for interactive planning

Usage:
    ros2 launch arm_system_bringup moveit_gazebo.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description with all nodes and arguments."""

    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with MoveIt Motion Planning plugin",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_gazebo"), "worlds", "lab.sdf"]
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

    # Get launch configurations
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world")
    log_level = LaunchConfiguration("log_level")

    # Build MoveIt configuration
    # Note: We don't set use_sim here because spawn_arm.launch.py handles robot_state_publisher
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"]
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,  # arm_world.launch.py already publishes this
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 1. Launch Gazebo with the arm (includes robot_state_publisher and spawn)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_gazebo"), "launch", "arm_world.launch.py"]
            )
        ),
        launch_arguments={
            "world": world_file,
        }.items(),
    )

    # 2. Load ros2_controllers configuration
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    # 3. Controller spawners - spawn the controllers needed for MoveIt
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Delay controller spawning to ensure Gazebo ros2_control plugin is ready
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,  # Wait for Gazebo to fully initialize
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller = TimerAction(
        period=4.0,  # Wait for joint_state_broadcaster to be active
        actions=[arm_controller_spawner],
    )

    # 4. MoveGroup node - delay to ensure controllers are spawned and active
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
        period=6.0,  # Wait for controllers to be fully active
        actions=[move_group_node],
    )

    # 3. RViz with MoveIt Motion Planning plugin
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    # Delay RViz start to ensure MoveGroup is ready
    delayed_rviz = TimerAction(
        period=8.0,  # Wait for MoveGroup to be fully initialized
        actions=[rviz_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_launch,
            delayed_joint_state_broadcaster,
            delayed_arm_controller,
            delayed_move_group,
            delayed_rviz,
        ]
    )
