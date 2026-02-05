from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Packages
    pkg_arm_description = FindPackageShare("arm_description")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.85", description="Z position (0.85 puts table legs on ground)")

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

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Controller spawner nodes using timers
    # Note: No separate ros2_control_node needed - Gazebo provides controller_manager via GazeboSimROS2ControlPlugin
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = TimerAction(
        period=4.0,  # wait for Gazebo's controller_manager to be ready
        actions=[joint_state_broadcaster_spawner]
    )

    arm_controller_node = TimerAction(
        period=8.0,  # wait to ensure broadcaster is active
        actions=[arm_controller_spawner]
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
        ],
        output="screen",
    )

    hand_controller_node = TimerAction(
        period=10.0,  # wait to ensure arm controller is active
        actions=[hand_controller_spawner]
    )

    # Spawn robot in Gazebo
    spawn_entity = TimerAction(
        period=1.0,  # Wait 3 seconds for Gazebo to initialize
        actions=[Node(
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
        )]
    )

    # Camera bridge - bridge Gazebo rgbd_camera topics to ROS 2
    camera_bridge = TimerAction(
        period=2.0,  # Wait 2 seconds for robot sensors to initialize
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # RGB image and camera info
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                # Depth image and camera info
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                # Point cloud
                "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                # Limit queue depth to prevent message flooding
                "--ros-args",
                "--param", "qos_overrides./camera/depth_image.publisher.depth:=1",
                "--param", "qos_overrides./camera/image.publisher.depth:=1",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            remappings=[
                ("/camera/image", "/camera/color/image_raw"),
                ("/camera/camera_info", "/camera/color/camera_info"),
                ("/camera/depth_image", "/camera/depth/image_raw"),
                ("/camera/depth_camera_info", "/camera/depth/camera_info"),
            ],
        )]
    )

    # Static transform to fix Gazebo's frame naming (arm/base_link/camera -> camera_link)
    camera_frame_fix = TimerAction(
        period=3.0,
        actions=[Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "camera_link", "arm/base_link/camera"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg, x_arg, y_arg, z_arg,

        # Nodes
        robot_state_publisher,
        joint_state_broadcaster_node,
        arm_controller_node,
        hand_controller_node,
        spawn_entity,
        camera_bridge,
        camera_frame_fix,
    ])
