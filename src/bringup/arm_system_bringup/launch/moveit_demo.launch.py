#!/usr/bin/env python3
"""
MoveIt Demo Launch File for Humanoid Arm System
This launch file properly orchestrates all necessary nodes in the correct order
to avoid duplicate services and ensure clean startup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Setup function to access launch arguments and build launch description."""

    # Launch arguments
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Build MoveIt configuration
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
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Get parameters for ros2_control
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    nodes_to_start = []

    # 1. Static transform publisher (world -> base_link)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    nodes_to_start.append(static_tf_node)

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    nodes_to_start.append(robot_state_publisher_node)

    # 3. ros2_control node (controller manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )
    nodes_to_start.append(ros2_control_node)

    # 4. Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    nodes_to_start.append(joint_state_broadcaster_spawner)

    # 5. Arm controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    nodes_to_start.append(arm_controller_spawner)

    # 6. MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_monitored_planning_scene": True},
        ],
    )
    nodes_to_start.append(move_group_node)

    # 7. RViz (optional)
    rviz_base = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )
    nodes_to_start.append(rviz_node)

    return nodes_to_start


def generate_launch_description():
    """Generate launch description with all nodes and arguments."""

    declared_arguments = []

    # Declare launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 for visualization",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "config", "moveit.rviz"]
            ),
            description="Path to RViz config file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
