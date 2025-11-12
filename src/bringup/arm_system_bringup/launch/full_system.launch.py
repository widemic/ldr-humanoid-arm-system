from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    world_arg = DeclareLaunchArgument(
        'simulation_world',
        default_value='',  # Set a default value if needed
        description='World file for the simulation'
    )

    # Define paths using FindPackageShare + PathJoinSubstitution
    headless_sim_path = PathJoinSubstitution([
        FindPackageShare('arm_gazebo'),
        'launch',
        'headless_sim.launch.py'
    ])

    control_path = PathJoinSubstitution([
        FindPackageShare('arm_control'),
        'launch',
        'control.launch.py'
    ])

    planner_path = PathJoinSubstitution([
        FindPackageShare('arm_moveit_config'),
        'launch',
        'planner.launch.py'
    ])

    # Include each launch description
    headless_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(headless_sim_path),
        launch_arguments={
            'simulation_world': LaunchConfiguration('simulation_world')
        }.items()
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_path)
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_path)
    )

    return LaunchDescription([
        world_arg,
        control,
        headless_sim,
        # planner
    ])

