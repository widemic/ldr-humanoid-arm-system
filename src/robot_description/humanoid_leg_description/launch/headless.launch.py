#!/usr/bin/env python3
"""
Headless launch for the humanoid robot (no GUIs).

- Starts Gazebo Harmonic server-only (-s) and auto-run (-r)
- Bridges /clock
- Spawns the humanoid robot
- Spawns joint_state_broadcaster and leg_controller
"""

import os
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
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("humanoid_leg_description"), "worlds", "empty.sdf"]
            ),
            description="Path to Gazebo world file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo",
        )
    )

    world_file = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_ros_gz_bridge = FindPackageShare("ros_gz_bridge")
    pkg_humanoid_leg_description = FindPackageShare("humanoid_leg_description")

    install_dir = get_package_prefix("humanoid_leg_description")
    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share')
    )

    # Set GZ_SIM_SYSTEM_PLUGIN_PATH to find gz_ros2_control plugin
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # gz_resource_path = SetEnvironmentVariable(
    #     name="GZ_SIM_RESOURCE_PATH", value=os.path.join(install_dir, "share")
    # )

    # gz_plugin_path = SetEnvironmentVariable(
    #     name="GZ_SIM_SYSTEM_PLUGIN_PATH", value="/opt/ros/humble/lib"
    # )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_file, " -r -s"]}.items(),
    )

    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_bridge, "launch", "clock_bridge.launch"])
        ),
        launch_arguments={"bridge_name": "gz_clock_bridge"}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_humanoid_leg_description, "launch", "spawn_robot.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    control_path = PathJoinSubstitution([
        FindPackageShare('humanoid_leg_description'),
        'launch',
        'control.launch.py'
    ])
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_path)
    )

    delayed_leg_controller = TimerAction(
        period=4.0,
        actions=[control],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_resource_path,
            gz_plugin_path,
            gazebo_server,
            clock_bridge,
            spawn_robot,
            delayed_leg_controller,
        ]
    )
