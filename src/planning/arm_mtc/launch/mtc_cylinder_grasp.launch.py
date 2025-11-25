from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Alternative launch file using the legacy moveit_gazebo.launch.py
    This launches:
    1. Gazebo with full visualization
    2. MoveIt with RViz
    3. MTC pick and place demo node
    """

    # Declare launch arguments
    simulation_world_arg = DeclareLaunchArgument(
        'simulation_world',
        default_value='',
        description='Optional world file for the simulation (uses default if empty)'
    )

    # ========== BUILD MOVEIT CONFIGURATION ==========
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"])
        .to_moveit_configs()
    )

    # ========== LAUNCH MOVEIT + GAZEBO (with GUI) ==========
    moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_system_bringup'),
                'launch',
                'moveit_gazebo.launch.py'
            ])
        )
    )

    # ========== LAUNCH MTC PICK AND PLACE NODE ==========
    # Wait 15 seconds for full system initialization
    mtc_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='arm_mtc',
                executable='mtc_pick_place_cylinder',
                name='mtc_pick_place_cylinder',
                output='screen',
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        simulation_world_arg,

        # Launch sequence
        moveit_gazebo_launch,    # Start full system with GUI
        mtc_node,                # Start MTC demo after 15s
    ])
