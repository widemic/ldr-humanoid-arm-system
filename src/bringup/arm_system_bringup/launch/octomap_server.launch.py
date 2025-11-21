from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/camera/depth/points",
        description="Input PointCloud2 topic for building the OctoMap."
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_link",
        description="Fixed frame used for the OctoMap."
    )

    base_frame_id_arg = DeclareLaunchArgument(
        "base_frame_id",
        default_value="base_link",
        description="Robot base frame for TF lookups."
    )

    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Voxel size in meters for the OctoMap."
    )

    max_range_arg = DeclareLaunchArgument(
        "max_range",
        default_value="5.0",
        description="Maximum sensor range to integrate (meters)."
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo/Sim time."
    )

    default_map_file = PathJoinSubstitution([
        FindPackageShare("arm_system_bringup"),
        "config",
        "empty_map.bt",
    ])

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=default_map_file,
        description="Path to a .bt map file to preload (defaults to packaged empty map)."
    )

    use_color_arg = DeclareLaunchArgument(
        "use_color_octomap",
        default_value="true",
        description="Use color_octomap_server_node to store RGB data in the octree."
    )

    config_path = PathJoinSubstitution([
        FindPackageShare("arm_system_bringup"),
        "config",
        "octomap_server.yaml",
    ])

    color_octomap_server = Node(
        package="octomap_server",
        executable="color_octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            config_path,
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "resolution": LaunchConfiguration("resolution"),
                "sensor_model/max_range": LaunchConfiguration("max_range"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file": LaunchConfiguration("map_file"),
            },
        ],
        remappings=[
            ("cloud_in", LaunchConfiguration("pointcloud_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("use_color_octomap")),
    )

    grayscale_octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            config_path,
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "resolution": LaunchConfiguration("resolution"),
                "sensor_model/max_range": LaunchConfiguration("max_range"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file": LaunchConfiguration("map_file"),
            },
        ],
        remappings=[
            ("cloud_in", LaunchConfiguration("pointcloud_topic")),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_color_octomap")),
    )

    return LaunchDescription(
        [
            pointcloud_topic_arg,
            frame_id_arg,
            base_frame_id_arg,
            resolution_arg,
            max_range_arg,
            use_sim_time_arg,
            map_file_arg,
            use_color_arg,
            color_octomap_server,
            grayscale_octomap_server,
        ]
    )
