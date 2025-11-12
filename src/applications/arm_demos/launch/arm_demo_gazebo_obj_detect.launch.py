#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find package directories
    arm_system_bringup_dir = get_package_share_directory('arm_system_bringup')
    arm_perception_dir = get_package_share_directory('arm_perception')
    
    # 1. Start full system
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_system_bringup_dir, 'launch', 'full_system.launch.py')
        )
    )
    
    # 2. Open Gazebo GUI after 5 seconds
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
    
    # 3. Open Object Detection Node after 10 seconds (increased delay)
    obj_detect = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='arm_perception',  # Corrected package name (removed underscore)
                executable='object_recognition_node.py',  # Assuming this is the executable name
                name='object_recognition_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 4 Open RViz with deep_camera config from arm_perception
    rviz_launch = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(arm_perception_dir, 'config', 'deep_camera.rviz')],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        full_system_launch,
        gazebo_gui,
        obj_detect,
        rviz_launch,
        
        # Just logs
        launch.actions.LogInfo(msg="Arm Demo Launcher started"),
        launch.actions.LogInfo(msg="Launch sequence: full_system -> Gazebo -> RViz with deep_camera"),
        launch.actions.LogInfo(msg="RViz config: deep_camera.rviz from arm_perception package"),
    ])