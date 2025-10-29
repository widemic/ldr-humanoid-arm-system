from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = PathJoinSubstitution([pkg_arm_description, 'urdf', 'arm.xacro'])

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arm',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        robot_state_publisher,
        spawn_entity
    ])