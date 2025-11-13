from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock if available",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    # MoveIt config (robot_description, SRDF, kinematics, limits)
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml = PathJoinSubstitution([FindPackageShare("arm_moveit_config"), "config", "servo.yaml"])
    dualsense_yaml = PathJoinSubstitution([FindPackageShare("arm_teleop"), "config", "dualsense.yaml"])

    return LaunchDescription([
        declare_use_sim_time,
        # Joystick driver
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            output="screen",
            parameters=[{'autorepeat_rate': 0.0}, {"use_sim_time": use_sim_time}],
        ),
        # MoveIt Servo (now has robot_description + servo.yaml)
        # Node(
        #     package="moveit_servo",
        #     executable="servo_node",
        #     name="moveit_servo",
        #     output="screen",
        #     parameters=[
        #         moveit_config.to_dict(),
        #         servo_yaml,
        #         {"moveit_servo.move_group_name": "arm"},
        #         {"use_sim_time": use_sim_time},
        #     ],
        # ),
        # DualSense mapper (now with ROS 2 param file format)
        Node(
            package="arm_teleop",
            executable="dualsense_teleop.py",
            name="dualsense_teleop",
            output="screen",
            parameters=[dualsense_yaml, {"use_sim_time": use_sim_time}],
        ),
    ])
