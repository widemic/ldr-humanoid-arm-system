"""
Launch file for MoveIt Task Constructor demo

Prerequisites:
    1. Launch Gazebo simulation:
       ros2 launch arm_control sim.launch.py

    2. Launch MoveIt (without RViz):
       ros2 launch arm_moveit_config demo.launch.py use_rviz:=false

    3. Then launch this file:
       ros2 launch arm_mtc mtc_demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # MTC Demo Node
    mtc_demo_node = Node(
        package='arm_mtc',
        executable='pick_place_demo.py',
        name='pick_place_demo',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mtc_demo_node
    ])
