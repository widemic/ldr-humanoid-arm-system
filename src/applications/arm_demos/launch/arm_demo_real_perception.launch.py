#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Setează variabile de mediu
    env_vars = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ]
    
    # Argumente de launch
    camera_delay = LaunchConfiguration('camera_delay', default='3.0')
    rviz_delay = LaunchConfiguration('rviz_delay', default='8.0')
    
    # Find package directories
    arm_description_dir = get_package_share_directory('arm_description')
    arm_perception_dir = get_package_share_directory('arm_perception')
    
    urdf_path = PathJoinSubstitution([arm_description_dir, 'urdf', 'arm.urdf.xacro'])
    
    # Robot description
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_path, ' ',
            'use_sim:=false'
        ]),
        value_type=str
    )
    
    return LaunchDescription([
        # Variabile de mediu
        *env_vars,
        
        # Argumente
        DeclareLaunchArgument(
            'camera_delay',
            default_value='3.0',
            description='Delay before starting external camera (seconds)'
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='8.0',
            description='Delay before starting RViz (seconds)'
        ),
        
        # 1. Static Transform Publisher - CRITIC PENTRU POINTCLOUD
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=['0', '-1.0', '1.0', '-1.57', '0.0', '-1.57', 'base_fixture_link', 'camera_link'],
            output='screen'
        ),
        
        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # 3. Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'source_list': ['joint_states']
            }]
        ),
        
        # 4. External camera node
        TimerAction(
            period=LaunchConfiguration('camera_delay'),
            actions=[
                Node(
                    package='perception_tests',
                    executable='external_camera_node.py',
                    name='external_camera_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    remappings=[
                        ('/camera/depth/points', '/camera/points'),
                        ('/camera/depth/camera_info', '/camera/camera_info')
                    ]
                )
            ]
        ),
        
        # 5. RViz
        TimerAction(
            period=LaunchConfiguration('rviz_delay'),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(arm_perception_dir, 'config', 'deep_real_camera.rviz')],
                    parameters=[{'use_sim_time': False}]
                )
            ]
        ),
        
        # 6. Open Object Detection Node after 1 seconds
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='arm_perception',  
                    executable='object_recognition_node.py', 
                    name='object_recognition_node',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),
        # Logs
        launch.actions.LogInfo(msg="=== Arm Description + External Camera Demo ==="),
        launch.actions.LogInfo(msg="Added static transform from base_fixture_link to camera_link"),
        launch.actions.LogInfo(msg="PointCloud should now be visible in RViz with base_fixture_link fixed frame"),
    ])