from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_arm_description = FindPackageShare("arm_description")
    # Paths
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_arm_description, "urdf", "arm.urdf.xacro"]),
        " use_sim:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controllers = PathJoinSubstitution([
        FindPackageShare("arm_control"),
        "config",
        "controllers.yaml"
    ])

    # Wrap Xacro command output as string parameter
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description,{"use_sim_time": True}],
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers, {"robot_description": robot_description}],
        output="screen"
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        rsp,
        # controller_manager,
        spawner_jsb,
        spawner_arm
    ])
