#!/usr/bin/env python3
"""
Dynamic Object Tracker Node

Tracks moving objects from point cloud, assigns persistent IDs, and updates
MoveIt planning scene with collision objects that follow the objects as they move.

This solves the "ghost objects" problem by:
1. Clustering point cloud into objects
2. Tracking objects across frames (persistent IDs)
3. Removing old collision objects before adding new ones
4. Publishing updated collision objects to MoveIt planning scene

Topics:
    Subscribed:
        - /camera/depth/points (sensor_msgs/PointCloud2)
    Published:
        - /planning_scene (moveit_msgs/PlanningScene)
        - /tracked_objects (visualization_msgs/MarkerArray) - debug

Author: Auto-generated for dynamic object tracking
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import numpy as np
try:
    from scipy.spatial.distance import cdist
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class DynamicObjectTracker(Node):
    """Tracks moving objects and updates MoveIt planning scene in real-time."""

    def __init__(self):
        super().__init__('dynamic_object_tracker')

        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('min_cluster_size', 50)  # Minimum points per object
        self.declare_parameter('max_cluster_size', 5000)  # Maximum points per object
        self.declare_parameter('cluster_tolerance', 0.05)  # 5cm clustering distance
        self.declare_parameter('max_tracking_distance', 0.3)  # 30cm max movement between frames
        self.declare_parameter('object_timeout', 2.0)  # Remove objects not seen for 2s
        self.declare_parameter('min_object_height', 0.02)  # 2cm minimum height
        self.declare_parameter('reference_frame', 'base_link')

        # Get parameters
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.max_tracking_distance = self.get_parameter('max_tracking_distance').value
        self.object_timeout = self.get_parameter('object_timeout').value
        self.min_object_height = self.get_parameter('min_object_height').value
        self.reference_frame = self.get_parameter('reference_frame').value

        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            qos_profile
        )

        # Publishers
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos_profile
        )

        self.debug_markers_pub = self.create_publisher(
            MarkerArray,
            '/tracked_objects',
            qos_profile
        )

        # Tracking state
        self.tracked_objects = {}  # {id: {'centroid': np.array, 'bbox': tuple, 'last_seen': time}}
        self.next_object_id = 0
        self.collision_object_ids = set()  # Track collision objects we've published

        # Statistics
        self.frame_count = 0
        self.last_point_count = 0

        self.get_logger().info('Dynamic Object Tracker started')
        self.get_logger().info(f'  - Min cluster size: {self.min_cluster_size} points')
        self.get_logger().info(f'  - Cluster tolerance: {self.cluster_tolerance}m')
        self.get_logger().info(f'  - Max tracking distance: {self.max_tracking_distance}m')
        self.get_logger().info(f'  - Object timeout: {self.object_timeout}s')
        self.get_logger().info(f'  - Scipy available: {SCIPY_AVAILABLE}')

    def pointcloud_callback(self, msg: PointCloud2):
        """Process point cloud and update tracked objects."""
        # Convert point cloud to numpy array
        points = self.pointcloud2_to_array(msg)
        if points is None or len(points) < self.min_cluster_size:
            return

        # Filter ground plane (remove points below certain height)
        # IMPORTANT: This removes the table/floor from detection
        original_count = len(points)
        points = points[points[:, 2] > self.min_object_height]

        if len(points) < self.min_cluster_size:
            if self.frame_count % 30 == 0:
                self.get_logger().warn(
                    f'All points filtered out! Original: {original_count}, '
                    f'After ground filter: {len(points)}. Try lowering min_object_height.'
                )
            return

        # Additional filtering: remove very far points
        # Keep only points within a reasonable workspace
        points = points[np.abs(points[:, 0]) < 3.0]  # X: -3 to 3m
        points = points[np.abs(points[:, 1]) < 3.0]  # Y: -3 to 3m
        points = points[points[:, 2] < 3.0]          # Z: below 3m

        if len(points) < self.min_cluster_size:
            return

        # Cluster points into objects
        clusters = self.cluster_points(points)

        # Extract object features (centroid, bounding box)
        current_objects = []
        for cluster in clusters:
            if len(cluster) < self.min_cluster_size or len(cluster) > self.max_cluster_size:
                continue

            centroid = np.mean(cluster, axis=0)
            bbox = self.compute_bounding_box(cluster)
            current_objects.append({
                'centroid': centroid,
                'bbox': bbox,
                'points': cluster
            })

        # Track objects (associate with previous frame)
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] / 1e9

        tracked_ids = self.associate_objects(current_objects, current_time)

        # Remove stale objects
        self.remove_stale_objects(current_time)

        # Publish collision objects to planning scene
        self.publish_collision_objects(msg.header.frame_id)

        # Publish debug markers
        self.publish_debug_markers(msg.header.frame_id)

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array using sensor_msgs_py."""
        try:
            # Use sensor_msgs_py for proper conversion
            points_list = []

            # Read points using pc2.read_points
            for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point
                # Additional validation
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    # Filter out very distant points
                    if abs(x) < 10.0 and abs(y) < 10.0 and abs(z) < 10.0:
                        points_list.append([x, y, z])

            if not points_list:
                return None

            points_array = np.array(points_list, dtype=np.float32)

            # Debug info
            self.frame_count += 1
            self.last_point_count = len(points_array)
            if self.frame_count % 30 == 0:  # Log every 30 frames (~1s)
                self.get_logger().info(f'Frame {self.frame_count}: {len(points_array)} valid points')

            return points_array

        except Exception as e:
            self.get_logger().error(f'Failed to convert point cloud: {e}', throttle_duration_sec=1.0)
            return None

    def cluster_points(self, points):
        """Improved Euclidean clustering with better performance."""
        if len(points) == 0:
            return []

        clusters = []
        remaining = set(range(len(points)))
        cluster_count = 0

        while remaining and cluster_count < 10:  # Limit to 10 clusters for performance
            # Start new cluster with first remaining point
            seed_idx = remaining.pop()
            cluster = [seed_idx]
            to_check = [seed_idx]

            # Region growing
            while to_check and len(cluster) < self.max_cluster_size:
                idx = to_check.pop(0)  # FIFO for breadth-first search
                point = points[idx]

                # Find neighbors within tolerance (optimized)
                neighbors_to_remove = []
                for other_idx in remaining:
                    other_point = points[other_idx]
                    dist = np.linalg.norm(point - other_point)

                    if dist < self.cluster_tolerance:
                        cluster.append(other_idx)
                        to_check.append(other_idx)
                        neighbors_to_remove.append(other_idx)

                        # Early stop if cluster is getting too large
                        if len(cluster) >= self.max_cluster_size:
                            break

                # Remove neighbors from remaining set
                for neighbor_idx in neighbors_to_remove:
                    remaining.remove(neighbor_idx)

            # Only keep clusters within size bounds
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                clusters.append(points[cluster])
                cluster_count += 1

                if self.frame_count % 30 == 0:
                    self.get_logger().debug(f'Cluster {cluster_count}: {len(cluster)} points')

        if self.frame_count % 30 == 0 and clusters:
            self.get_logger().info(f'Found {len(clusters)} clusters')

        return clusters

    def compute_bounding_box(self, points):
        """Compute axis-aligned bounding box."""
        min_point = np.min(points, axis=0)
        max_point = np.max(points, axis=0)
        dimensions = max_point - min_point
        center = (min_point + max_point) / 2.0
        return center, dimensions

    def associate_objects(self, current_objects, current_time):
        """Associate current detections with tracked objects."""
        if not current_objects:
            return []

        if not self.tracked_objects:
            # First frame - initialize tracking
            for obj in current_objects:
                obj_id = self.next_object_id
                self.next_object_id += 1
                self.tracked_objects[obj_id] = {
                    'centroid': obj['centroid'],
                    'bbox': obj['bbox'],
                    'last_seen': current_time
                }
            self.get_logger().info(f'Initialized {len(current_objects)} objects')
            return list(self.tracked_objects.keys())

        # Extract centroids
        current_centroids = np.array([obj['centroid'] for obj in current_objects])
        tracked_centroids = np.array([obj['centroid'] for obj in self.tracked_objects.values()])
        tracked_ids = list(self.tracked_objects.keys())

        # Compute distances (use scipy if available, else manual)
        if SCIPY_AVAILABLE:
            distances = cdist(current_centroids, tracked_centroids)
        else:
            # Manual distance calculation
            distances = np.zeros((len(current_centroids), len(tracked_centroids)))
            for i, curr in enumerate(current_centroids):
                for j, track in enumerate(tracked_centroids):
                    distances[i, j] = np.linalg.norm(curr - track)

        # Hungarian assignment (greedy for simplicity)
        assigned_ids = []
        used_tracked = set()
        updated_count = 0
        new_count = 0

        for i, obj in enumerate(current_objects):
            # Find closest tracked object
            min_dist_idx = np.argmin(distances[i])
            min_dist = distances[i][min_dist_idx]

            if min_dist < self.max_tracking_distance and min_dist_idx not in used_tracked:
                # Update existing object
                obj_id = tracked_ids[min_dist_idx]
                old_centroid = self.tracked_objects[obj_id]['centroid']
                self.tracked_objects[obj_id] = {
                    'centroid': obj['centroid'],
                    'bbox': obj['bbox'],
                    'last_seen': current_time
                }
                assigned_ids.append(obj_id)
                used_tracked.add(min_dist_idx)
                updated_count += 1

                # Log movement
                movement = np.linalg.norm(obj['centroid'] - old_centroid)
                if movement > 0.01:  # More than 1cm
                    self.get_logger().info(
                        f'Object {obj_id} moved {movement:.3f}m to [{obj["centroid"][0]:.2f}, '
                        f'{obj["centroid"][1]:.2f}, {obj["centroid"][2]:.2f}]',
                        throttle_duration_sec=1.0
                    )
            else:
                # New object
                obj_id = self.next_object_id
                self.next_object_id += 1
                self.tracked_objects[obj_id] = {
                    'centroid': obj['centroid'],
                    'bbox': obj['bbox'],
                    'last_seen': current_time
                }
                assigned_ids.append(obj_id)
                new_count += 1
                self.get_logger().info(f'New object {obj_id} detected at {obj["centroid"]}')

        if updated_count > 0 or new_count > 0:
            self.get_logger().debug(f'Updated {updated_count} objects, added {new_count} new objects')

        return assigned_ids

    def remove_stale_objects(self, current_time):
        """Remove objects not seen recently."""
        to_remove = []
        for obj_id, obj_data in self.tracked_objects.items():
            if current_time - obj_data['last_seen'] > self.object_timeout:
                to_remove.append(obj_id)

        for obj_id in to_remove:
            del self.tracked_objects[obj_id]
            self.get_logger().info(f'Removed stale object {obj_id}')

    def publish_collision_objects(self, frame_id):
        """Publish collision objects to MoveIt planning scene."""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        # Remove old collision objects first
        for old_id in self.collision_object_ids:
            co = CollisionObject()
            co.id = f"tracked_object_{old_id}"
            co.operation = CollisionObject.REMOVE
            planning_scene.world.collision_objects.append(co)

        # Clear the set
        self.collision_object_ids.clear()

        # Add current tracked objects
        for obj_id, obj_data in self.tracked_objects.items():
            center, dimensions = obj_data['bbox']

            # Skip very small objects
            if np.max(dimensions) < self.min_object_height:
                continue

            co = CollisionObject()
            co.header = Header()
            co.header.frame_id = self.reference_frame
            co.header.stamp = self.get_clock().now().to_msg()
            co.id = f"tracked_object_{obj_id}"

            # Create box primitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(d) for d in dimensions]

            # Set pose
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])
            pose.orientation.w = 1.0

            co.primitives.append(primitive)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD

            planning_scene.world.collision_objects.append(co)
            self.collision_object_ids.add(obj_id)

        # Publish planning scene
        self.planning_scene_pub.publish(planning_scene)

    def publish_debug_markers(self, frame_id):
        """Publish visualization markers for debugging."""
        marker_array = MarkerArray()

        for obj_id, obj_data in self.tracked_objects.items():
            center, dimensions = obj_data['bbox']

            # Bounding box marker
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = self.reference_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(dimensions[0])
            marker.scale.y = float(dimensions[1])
            marker.scale.z = float(dimensions[2])

            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "labels"
            text_marker.id = obj_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z += float(dimensions[2] / 2.0 + 0.1)
            text_marker.text = f"ID: {obj_id}"
            text_marker.scale.z = 0.1
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker_array.markers.append(text_marker)

        self.debug_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObjectTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Dynamic Object Tracker')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
