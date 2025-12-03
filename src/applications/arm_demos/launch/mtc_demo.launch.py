#!/usr/bin/env python3
"""
Launch file for MoveIt Task Constructor demos.

This launch file starts:
1. MoveIt move_group with MTC capabilities
2. RViz with MTC visualization panel
3. Option to run MTC demo scripts

Usage:
  # Launch with simple demo
  ros2 launch arm_demos mtc_demo.launch.py demo:=simple

  # Launch with pick-place demo
  ros2 launch arm_demos mtc_demo.launch.py demo:=pick_place

  # Launch MTC environment only (no demo script)
  ros2 launch arm_demos mtc_demo.launch.py demo:=none

  # Use with Gazebo simulation
  ros2 launch arm_demos mtc_demo.launch.py use_sim:=true

Arguments:
  - demo: Which demo to run (simple, pick_place, none) [default: simple]
  - use_sim: Use Gazebo simulation (true/false) [default: false]
  - use_rviz: Launch RViz (true/false) [default: true]
  - log_level: Logging level (DEBUG, INFO, WARN, ERROR) [default: INFO]

Author: LDR Robotics Team
License: MIT
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def check_mtc_installed():
    """Check if MTC packages are installed."""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            check=True
        )
        return 'moveit_task_constructor_core' in result.stdout
    except:
        return False


def launch_setup(context, *args, **kwargs):
    """Setup function to handle dynamic launch decisions."""

    # Get launch configurations
    demo = LaunchConfiguration('demo').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    launch_actions = []

    # Check if MTC is installed
    mtc_installed = check_mtc_installed()
    if demo in ['simple', 'pick_place'] and not mtc_installed:
        print("\n" + "="*60)
        print("WARNING: MoveIt Task Constructor not installed!")
        print("="*60)
        print("\nMTC demo cannot run without MTC packages.")
        print("\nTo install MTC, run:")
        print("  sudo apt install ros-jazzy-moveit-task-constructor-*")
        print("\nFor detailed installation check, run:")
        print("  ros2 run arm_demos check_mtc.py")
        print("\nLaunching MoveIt environment only (no demo)...")
        print("="*60 + "\n")
        demo = 'none'  # Override to skip demo

    # ============================================
    # Optional: Launch Gazebo simulation
    # ============================================
    if use_sim == 'true':
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('arm_control'),
                    'launch',
                    'sim.launch.py'
                ])
            ]),
            launch_arguments={
                'use_rviz': 'false',  # We'll launch RViz separately
            }.items()
        )
        launch_actions.append(gazebo_launch)

        # Delay for Gazebo to start
        launch_actions.append(TimerAction(period=5.0, actions=[]))

    # ============================================
    # Launch MoveIt move_group
    # ============================================

    # MoveIt move_group launch (without MTC capabilities for now)
    # TODO: Enable MTC capabilities when moveit_task_constructor packages are installed
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ])
    )
    launch_actions.append(moveit_launch)

    # ============================================
    # Launch RViz with MTC visualization
    # ============================================
    if use_rviz == 'true':
        # RViz config with MTC panel
        rviz_config_file = PathJoinSubstitution([
            FindPackageShare('arm_moveit_config'),
            'config',
            'moveit.rviz'
        ])

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
            parameters=[{
                'use_sim_time': use_sim == 'true',
            }],
        )

        # Delay RViz to let move_group start
        launch_actions.append(
            TimerAction(period=3.0, actions=[rviz_node])
        )

    # ============================================
    # Launch MTC demo script
    # ============================================
    if demo == 'simple':
        demo_node = Node(
            package='arm_demos',
            executable='mtc_simple_demo.py',
            name='mtc_simple_demo',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim == 'true',
            }],
            arguments=['--ros-args', '--log-level', log_level],
        )

        # Delay demo to let everything initialize
        launch_actions.append(
            TimerAction(period=8.0, actions=[demo_node])
        )

    elif demo == 'pick_place':
        demo_node = Node(
            package='arm_demos',
            executable='mtc_pick_place_demo.py',
            name='mtc_pick_place_demo',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim == 'true',
            }],
            arguments=['--ros-args', '--log-level', log_level],
        )

        # Delay demo to let everything initialize
        launch_actions.append(
            TimerAction(period=8.0, actions=[demo_node])
        )

    elif demo == 'none':
        # No demo script - just MTC environment
        pass

    else:
        print(f"Warning: Unknown demo type '{demo}'. No demo script will be launched.")

    return launch_actions


def generate_launch_description():
    """Generate the launch description."""

    # Declare arguments
    declare_demo_arg = DeclareLaunchArgument(
        'demo',
        default_value='simple',
        description='Which demo to run: simple, pick_place, or none'
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use Gazebo simulation'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )

    return LaunchDescription([
        declare_demo_arg,
        declare_use_sim_arg,
        declare_use_rviz_arg,
        declare_log_level_arg,
        OpaqueFunction(function=launch_setup)
    ])
