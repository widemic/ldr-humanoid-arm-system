"""
Launch MoveIt Servo with joystick teleoperation.

Sequence:
1. Move arm to ready position (via arm_controller action)
2. Unload arm_controller (release command interfaces)
3. Spawn servo_controller (forward_command)
4. Start MoveIt Servo + teleop

Prerequisites: full_system running (Gazebo + controllers)

Usage:
    ros2 launch arm_teleop servo_joystick_teleop.launch.py
    ros2 launch arm_teleop servo_joystick_teleop.launch.py debug_mode:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    debug_mode_arg = DeclareLaunchArgument("debug_mode", default_value="false")
    linear_scale_arg = DeclareLaunchArgument("linear_scale", default_value="0.3")
    angular_scale_arg = DeclareLaunchArgument("angular_scale", default_value="0.5")
    publish_rate_arg = DeclareLaunchArgument("publish_rate", default_value="50.0")

    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(
            file_path="config/arm_description.urdf.xacro",
            mappings={"use_sim": "true"},
        )
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo.move_group_name": "arm",
        "moveit_servo.planning_frame": "base_link",
        "moveit_servo.ee_frame_name": "end_effector_link",
        "moveit_servo.robot_link_command_frame": "base_link",
        "moveit_servo.command_in_type": "speed_units",
        "moveit_servo.scale.linear": LaunchConfiguration("linear_scale"),
        "moveit_servo.scale.rotational": LaunchConfiguration("angular_scale"),
        "moveit_servo.publish_period": PythonExpression(
            ["1.0 / ", LaunchConfiguration("publish_rate")]
        ),
        "moveit_servo.low_latency_mode": False,
        "moveit_servo.publish_joint_positions": True,
        "moveit_servo.publish_joint_velocities": False,
        "moveit_servo.publish_joint_accelerations": False,
        "moveit_servo.cartesian_command_in_topic": "~/delta_twist_cmds",
        "moveit_servo.joint_command_in_topic": "~/delta_joint_cmds",
        "moveit_servo.command_out_topic": "/servo_controller/commands",
        "moveit_servo.command_out_type": "std_msgs/Float64MultiArray",
        "moveit_servo.status_topic": "~/status",
        "moveit_servo.joint_topic": "/joint_states",
        "moveit_servo.check_collisions": False,
        "moveit_servo.collision_check_rate": 10.0,
        "moveit_servo.lower_singularity_threshold": 1e9,
        "moveit_servo.hard_stop_singularity_threshold": 1e10,
        "moveit_servo.leaving_singularity_threshold_multiplier": 1.0,
        "moveit_servo.joint_limit_margins": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
        "moveit_servo.incoming_command_timeout": PythonExpression(
            ["2.0 / ", LaunchConfiguration("publish_rate")]
        ),
        "moveit_servo.use_smoothing": False,
    }

    # Step 1: Move arm to ready position via arm_controller action
    move_to_ready = LogInfo(msg="Step 1: Moving arm to ready position...")
    move_to_ready_cmd = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            "{trajectory: {joint_names: [shoulder_pitch_joint, shoulder_roll_joint, shoulder_yaw_joint, elbow_pitch_joint, elbow_yaw_joint, wrist_roll_joint], points: [{positions: [0.1, 0.1, 0.1, -0.1, 0.1, 0.1], time_from_start: {sec: 2, nanosec: 0}}]}}",
        ],
        output="screen",
    )

    # Step 2: Unload arm_controller (fully release command interfaces)
    unload_arm = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="Step 2: Unloading arm_controller..."),
            ExecuteProcess(
                cmd=["ros2", "control", "set_controller_state", "arm_controller", "inactive"],
                output="screen",
            ),
        ],
    )
    unload_arm2 = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "unload_controller", "arm_controller"],
                output="screen",
            ),
        ],
    )

    # Step 3: Spawn servo_controller
    spawn_servo_ctrl = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="Step 3: Spawning servo_controller..."),
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", "servo_controller"],
                output="screen",
            ),
        ],
    )

    # Step 4: Start MoveIt Servo
    servo_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Step 4: Starting MoveIt Servo..."),
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="servo_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    servo_params,
                    {"use_sim_time": True},
                ],
            ),
        ],
    )

    # Joy node (start immediately)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            "autorepeat_rate": 50.0,
        }],
    )

    # Step 5: Teleop node
    servo_teleop_node = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg="Step 5: Servo teleop ready! Hold L1 and move joystick."),
            Node(
                package="arm_teleop",
                executable="servo_joystick_teleop.py",
                name="servo_joystick_teleop",
                output="screen",
                parameters=[{
                    "debug_mode": LaunchConfiguration("debug_mode"),
                    "linear_scale": LaunchConfiguration("linear_scale"),
                    "angular_scale": LaunchConfiguration("angular_scale"),
                    "servo_topic": "/servo_node/delta_twist_cmds",
                    "frame_id": "base_link",
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "deadzone": 0.1,
                }],
            ),
        ],
    )

    return LaunchDescription([
        debug_mode_arg,
        linear_scale_arg,
        angular_scale_arg,
        publish_rate_arg,
        # Sequence
        move_to_ready,
        move_to_ready_cmd,
        unload_arm,
        unload_arm2,
        spawn_servo_ctrl,
        joy_node,
        servo_node,
        servo_teleop_node,
    ])
