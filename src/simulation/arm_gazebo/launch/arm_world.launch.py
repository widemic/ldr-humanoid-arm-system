from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():
    # ROS 2 Jazzy uses Gazebo Harmonic (new Gazebo, formerly Ignition)
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge')

    # Get install prefix to resolve package:// URIs
    install_dir = get_package_prefix('arm_description')

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share')
    )

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_arm_gazebo, 'worlds', 'lab.sdf']),
        description='Path to the world file'
    )

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r']
        }.items()
    )

    # Spawn the arm model
    spawn_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_gazebo, 'launch', 'spawn_arm.launch.py'])
        )
    )

    # Clock bridge - essential for simulation time
    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_bridge, 'launch', 'clock_bridge.launch'])
        ),
        launch_arguments={
            'bridge_name': 'gz_clock_bridge' 
        }.items()
    )
    
    return LaunchDescription([
        gz_resource_path,
        world_arg,
        gazebo,
        clock_bridge,
        spawn_arm
    ])