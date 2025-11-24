#!/usr/bin/env python3
"""
Launch file for OpenCV Grasp Detector demo.

Usage:
    ros2 launch perception_tests opencv_grasp_demo.launch.py
    ros2 launch perception_tests opencv_grasp_demo.launch.py use_camera:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Launch external camera node (set to true if not running separately)'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization publishing'
    )

    min_area_arg = DeclareLaunchArgument(
        'min_object_area',
        default_value='1000',
        description='Minimum object area in pixels'
    )

    max_area_arg = DeclareLaunchArgument(
        'max_object_area',
        default_value='100000',
        description='Maximum object area in pixels'
    )

    # OpenCV Grasp Detector Node
    grasp_detector_node = Node(
        package='perception_tests',
        executable='opencv_grasp_detector.py',
        name='opencv_grasp_detector',
        output='screen',
        parameters=[{
            'min_object_area': LaunchConfiguration('min_object_area'),
            'max_object_area': LaunchConfiguration('max_object_area'),
            'depth_min': 0.3,
            'depth_max': 2.0,
            'visualize': LaunchConfiguration('visualize'),
            'camera_frame': 'camera_color_optical_frame',
        }],
        remappings=[
            # Remap if your camera topics have different names
            # ('/camera/color/image_raw', '/your_camera/rgb/image'),
            # ('/camera/depth/image_raw', '/your_camera/depth/image'),
        ]
    )

    # Optional: External Camera Node (if not running separately)
    camera_node = Node(
        package='perception_tests',
        executable='external_camera_node.py',
        name='external_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera'))
    )

    # Image View for visualization (optional)
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='grasp_visualization',
        arguments=['/grasp_detector/visualization'],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    return LaunchDescription([
        use_camera_arg,
        visualize_arg,
        min_area_arg,
        max_area_arg,
        grasp_detector_node,
        camera_node,
        # Uncomment to auto-launch image viewer
        image_view_node,
    ])
