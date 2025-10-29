from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': PathJoinSubstitution([pkg_arm_gazebo, 'worlds', 'lab.world'])}.items()
    )

    spawn_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_gazebo, 'launch', 'spawn_arm.launch.py'])
        )
    )

    return LaunchDescription([gazebo, spawn_arm])