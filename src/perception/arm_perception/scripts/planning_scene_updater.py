#!/usr/bin/env python3
"""
Planning Scene Updater Node

Subscribes to OctoMap and publishes it to MoveIt planning scene in real-time
for collision-aware motion planning.

Topics:
    Subscribed:
        - /octomap_binary (octomap_msgs/Octomap)
        - /octomap_full (octomap_msgs/Octomap)
    Published:
        - /planning_scene (moveit_msgs/PlanningScene)

Author: Auto-generated for real-time collision perception
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from std_msgs.msg import Header


class PlanningSceneUpdater(Node):
    """Updates MoveIt planning scene with OctoMap data in real-time."""

    def __init__(self):
        super().__init__('planning_scene_updater')

        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('update_rate', 5.0)  # Hz
        self.declare_parameter('octomap_frame', 'base_fixture_link')
        self.declare_parameter('use_binary_octomap', True)
        self.declare_parameter('clear_scene_on_update', True)  # Clear old collision objects

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.octomap_frame = self.get_parameter('octomap_frame').value
        use_binary = self.get_parameter('use_binary_octomap').value
        self.clear_scene = self.get_parameter('clear_scene_on_update').value

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        )

        # Subscribe to OctoMap topic
        octomap_topic = '/octomap_binary' if use_binary else '/octomap_full'
        self.octomap_sub = self.create_subscription(
            Octomap,
            octomap_topic,
            self.octomap_callback,
            qos_profile
        )

        # Publisher for planning scene updates
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos_profile
        )

        # Store latest octomap
        self.latest_octomap = None
        self.octomap_received = False

        # Create timer for periodic publishing
        update_period = 1.0 / self.update_rate
        self.timer = self.create_timer(update_period, self.publish_planning_scene)

        self.get_logger().info(f'Planning Scene Updater started')
        self.get_logger().info(f'  - Subscribing to: {octomap_topic}')
        self.get_logger().info(f'  - Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'  - Reference frame: {self.octomap_frame}')
        self.get_logger().info(f'  - Clear scene on update: {self.clear_scene}')
        self.get_logger().info(f'  - Free space clearing: ENABLED (via OctoMap raycasting)')

    def octomap_callback(self, msg: Octomap):
        """Store the latest octomap."""
        self.latest_octomap = msg
        if not self.octomap_received:
            self.octomap_received = True
            self.get_logger().info('First OctoMap received - starting planning scene updates')

    def publish_planning_scene(self):
        """Publish planning scene with current octomap.

        The OctoMap includes both occupied AND free space information thanks to:
        - publish_free_space: true in octomap_server.yaml
        - Raycasting from camera to obstacles marks intermediate voxels as FREE
        - When objects move, old voxels are marked FREE by new raycast paths
        """
        if not self.octomap_received or self.latest_octomap is None:
            return

        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True  # This is a differential update

        # IMPORTANT: is_diff=True means we only update the OctoMap
        # MoveIt will merge this with existing planning scene
        # Free voxels in OctoMap will clear old occupied voxels

        # Set header with current time
        planning_scene.world.octomap.header = Header()
        planning_scene.world.octomap.header.frame_id = self.octomap_frame
        planning_scene.world.octomap.header.stamp = self.get_clock().now().to_msg()

        # Add octomap to planning scene world
        # This OctoMap contains:
        # - Occupied voxels where obstacles are currently detected
        # - Free voxels where camera sees empty space (raycasting)
        # - Unknown voxels that haven't been observed yet
        planning_scene.world.octomap.octomap = self.latest_octomap

        # Publish the planning scene
        self.planning_scene_pub.publish(planning_scene)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneUpdater()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Planning Scene Updater')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
