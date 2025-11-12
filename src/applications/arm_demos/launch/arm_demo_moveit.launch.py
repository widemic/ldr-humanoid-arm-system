#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find package directories
    arm_system_bringup_dir = get_package_share_directory('arm_system_bringup')
    arm_moveit_config_dir = get_package_share_directory('arm_moveit_config')
    
    # 1. Start full system
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_system_bringup_dir, 'launch', 'full_system.launch.py')
        )
    )
    
    # 2. Open Gazebo GUI after 5 secondsa
    gazebo_gui = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                output='screen',
                name='gazebo_gui'
            )
        ]
    )
    
    # 3. Open MoveIT Demo
    moveit_demo_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(arm_moveit_config_dir, 'launch', 'demo.launch.py')
                )
            )
        ]
    )
    
    return LaunchDescription([
        full_system_launch,
        gazebo_gui,
        moveit_demo_launch,
        
        # Just logs
        launch.actions.LogInfo(msg="Arm Demo Launcher started"),
        launch.actions.LogInfo(msg="Launch sequence: full_system -> Gazebo -> MoveIt"),
    ])