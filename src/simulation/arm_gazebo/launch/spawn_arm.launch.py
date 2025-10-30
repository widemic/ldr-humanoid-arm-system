from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Packages
    pkg_arm_description = FindPackageShare("arm_description")
    pkg_arm_gazebo = FindPackageShare("arm_gazebo")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.0", description="Z position")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    # Robot description from XACRO
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_arm_description, "urdf", "arm.urdf.xacro"]),
        " use_sim:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controller YAML
    robot_controllers = PathJoinSubstitution(
        [pkg_arm_gazebo, "config", "controllers.yaml"]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Controller Manager
    controller_manager = TimerAction(
        period=5.0,  # 5 second delay for controller_manager
        actions=[Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
            output="screen",
        )]
    )
    
    # Controller spawner nodes using timers
    joint_state_broadcaster_node = TimerAction(
        period=10.0,  # wait 10 seconds after launch
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    arm_controller_node = TimerAction(
        period=15.0,  # wait 15 seconds to ensure broadcaster is active
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "arm",
            "-topic", "robot_description",
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
        ],
        output="screen",
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg, x_arg, y_arg, z_arg,

        # Nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_node,
        arm_controller_node,
        spawn_entity,
    ])
