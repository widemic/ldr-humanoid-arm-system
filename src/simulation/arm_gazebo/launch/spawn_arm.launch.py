from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Z position')

    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')

    # Process XACRO to get robot description (with use_sim:=true for Gazebo)
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_arm_description, 'urdf', 'arm.urdf.xacro']),
        ' use_sim:=true'
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher - publishes TF and robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo Harmonic using ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'arm',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher,
        spawn_entity
    ])