from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Launch file that integrates:
    1. Full system (headless Gazebo + controllers)
    2. MoveIt move_group for motion planning
    3. MTC pick and place demo node
    4. Optional RViz visualization
    """

    # Declare launch arguments
    simulation_world_arg = DeclareLaunchArgument(
        'simulation_world',
        default_value='',
        description='Optional world file for the simulation (uses default if empty)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
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

    # ========== LAUNCH FULL SYSTEM (Headless Gazebo + Controllers) ==========
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_system_bringup'),
                'launch',
                'full_system.launch.py'
            ])
        ),
        launch_arguments={
            'simulation_world': LaunchConfiguration('simulation_world')
        }.items()
    )

    # ========== LAUNCH MOVEIT MOVE_GROUP ==========
    # Wait 8 seconds for Gazebo and controllers to fully initialize
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('arm_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    ])
                )
            )
        ]
    )

    # ========== LAUNCH RVIZ (Optional) ==========
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arm_moveit_config'),
        'config',
        'moveit.rviz'
    ])

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
                parameters=[
                    {'use_sim_time': True}
                ],
                condition=IfCondition(LaunchConfiguration('use_rviz'))
            )
        ]
    )

    # ========== LAUNCH MTC PICK AND PLACE NODE ==========
    # Wait 12 seconds for MoveIt to fully initialize
    mtc_node = TimerAction(
        period=12.0,
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
        use_rviz_arg,

        # Launch sequence
        full_system_launch,      # 0s: Start Gazebo + controllers
        moveit_launch,           # 8s: Start MoveIt
        rviz_node,              # 10s: Start RViz (optional)
        mtc_node,               # 12s: Start MTC demo
    ])
