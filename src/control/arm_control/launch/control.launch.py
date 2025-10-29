from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindPackageShare

def generate_launch_description():
    pkg_arm_control = FindPackageShare('arm_control')

    controllers_yaml = PathJoinSubstitution([pkg_arm_control, 'config', 'controllers.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen'
    )

    return LaunchDescription([controller_manager])
