#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dual_arm_description")

    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "dual_arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "view_robot.rviz"])

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock from Gazebo (true) or system clock (false)",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start joint_state_publisher_gui",
    )
    use_rviz_config_arg = DeclareLaunchArgument(
        "use_rviz_config",
        default_value="true",
        description="Use saved RViz configuration file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    use_rviz_config = LaunchConfiguration("use_rviz_config")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(Command(["xacro ", urdf_file]), value_type=str),
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui),
    )

    rviz_with_config = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz_config),
    )

    rviz_default = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_rviz_config),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gui_arg,
            use_rviz_config_arg,
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            rviz_default,
            rviz_with_config,
        ]
    )
