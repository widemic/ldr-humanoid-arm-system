"""Launch visual teleop with skeleton tracking."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    hand_topic_arg = DeclareLaunchArgument(
        "hand_topic",
        default_value="/skeleton/hand_right",
        description="Topic for hand position from skeleton tracker",
    )
    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="false",
        description="Enable debug output and topic publishing",
    )
    enabled_arg = DeclareLaunchArgument(
        "enabled",
        default_value="true",
        description="Enable visual teleop on startup",
    )

    # Visual teleop node
    visual_teleop_node = Node(
        package="arm_teleop",
        executable="visual_teleop.py",
        name="visual_teleop",
        output="screen",
        parameters=[{
            "hand_topic": LaunchConfiguration("hand_topic"),
            "debug_mode": LaunchConfiguration("debug_mode"),
            "enabled": LaunchConfiguration("enabled"),
            "command_topic": "teleop/joint_commands",
            "base_frame": "base_link",
            "end_effector_frame": "end_effector_link",
            # Hand workspace (camera frame)
            "hand_workspace.x_min": -0.3,
            "hand_workspace.x_max": 0.3,
            "hand_workspace.y_min": -0.3,
            "hand_workspace.y_max": 0.3,
            "hand_workspace.z_min": 0.3,
            "hand_workspace.z_max": 1.2,
            # Robot workspace (base_link frame)
            "robot_workspace.x_min": 0.1,
            "robot_workspace.x_max": 0.4,
            "robot_workspace.y_min": -0.25,
            "robot_workspace.y_max": 0.25,
            "robot_workspace.z_min": -0.2,
            "robot_workspace.z_max": 0.3,
        }],
    )

    # Joint teleop backend node
    joint_teleop_node = Node(
        package="arm_teleop",
        executable="joint_teleop_node",
        name="joint_teleop_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "trajectory_duration": 0.15,
        }],
    )

    return LaunchDescription([
        hand_topic_arg,
        debug_mode_arg,
        enabled_arg,
        joint_teleop_node,
        visual_teleop_node,
    ])
