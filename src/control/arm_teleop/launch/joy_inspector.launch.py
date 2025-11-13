from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy_linux', executable='joy_linux_node', name='joy_node', output='screen'),
        Node(package='arm_teleop', executable='joy_inspector.py', name='joy_inspector', output='screen')
    ])