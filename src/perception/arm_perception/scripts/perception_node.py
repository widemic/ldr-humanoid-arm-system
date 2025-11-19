#!/usr/bin/env python3
"""
3D Perception Node for Geometric Shape Detection

This node processes point cloud data from an RGBD camera to detect geometric shapes
(cylinders and boxes) and automatically adds them as collision objects to the MoveIt
planning scene.

Based on automatic

addison's mycobot_ros2 perception pipeline.

Workflow:
1. Subscribe to camera point cloud (/camera/depth/points)
2. Transform to robot base frame (base_fixture_link)
3. Segment support plane (table/surface) using RANSAC
4. Cluster remaining points
5. Detect geometric primitives (cylinders, boxes)
6. Publish collision objects to MoveIt planning scene

Author: Generated for LDR Humanoid Arm System
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
import tf2_sensor_msgs
import tf2_geometry_msgs

# Optional: sklearn for RANSAC and clustering
try:
    from sklearn.linear_model import RANSACRegressor
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


class PerceptionNode(Node):
    """3D perception node for geometric shape detection and MoveIt scene generation"""

    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/depth/points')
        self.declare_parameter('target_frame', 'base_fixture_link')
        self.declare_parameter('plane_distance_threshold', 0.01)  # 1cm for RANSAC plane fitting
        self.declare_parameter('cluster_tolerance', 0.05)  # 5cm for clustering (increased for better grouping)
        self.declare_parameter('min_cluster_size', 200)  # Minimum points per cluster (increased to filter noise)
        self.declare_parameter('max_cluster_size', 10000)  # Maximum points per cluster
        self.declare_parameter('cylinder_radius_min', 0.01)  # 1cm minimum cylinder radius
        self.declare_parameter('cylinder_radius_max', 0.15)  # 15cm maximum cylinder radius
        self.declare_parameter('min_object_height', 0.03)  # 3cm minimum object height (balanced filtering)
        self.declare_parameter('processing_rate', 1.0)  # Hz - how often to process
        self.declare_parameter('point_downsample_factor', 4)  # Downsample every Nth point for speed
        self.declare_parameter('z_min', 0.0)  # Minimum Z in target frame (10cm above ground - filters floor)
        self.declare_parameter('z_max', 1.5)  # Maximum Z in target frame

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.plane_threshold = self.get_parameter('plane_distance_threshold').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.cylinder_radius_min = self.get_parameter('cylinder_radius_min').value
        self.cylinder_radius_max = self.get_parameter('cylinder_radius_max').value
        self.min_height = self.get_parameter('min_object_height').value
        self.processing_rate = self.get_parameter('processing_rate').value
        self.downsample_factor = self.get_parameter('point_downsample_factor').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS profile matching Gazebo camera publisher (RELIABLE)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.camera_topic,
            self.pointcloud_callback,
            qos_profile
        )

        # Publisher for collision objects
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/detected_objects',
            10
        )

        # Processing timer
        self.processing_timer = self.create_timer(
            1.0 / self.processing_rate,
            self.process_latest_pointcloud
        )

        # Latest point cloud storage
        self.latest_pc = None
        self.object_id_counter = 0
        self.current_object_ids = []  # Track current objects for deletion

        # Clear any existing detected objects from planning scene on startup
        self.startup_timer = self.create_timer(1.0, self.clear_all_objects_once)

        self.get_logger().info(f'Perception node started!')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info(f'Processing rate: {self.processing_rate} Hz')

        if not SKLEARN_AVAILABLE:
            self.get_logger().warn('sklearn not available - using simplified algorithms')

    def clear_all_objects_once(self):
        """Clear all detected objects from planning scene (called once on startup)"""
        # Remove objects with common prefixes that might exist from previous runs
        for i in range(100):  # Clear up to 100 potential old objects
            for obj_type in ['cylinder', 'box']:
                remove_obj = CollisionObject()
                remove_obj.header.frame_id = self.target_frame
                remove_obj.header.stamp = self.get_clock().now().to_msg()
                remove_obj.id = f"detected_{obj_type}_{i}"
                remove_obj.operation = CollisionObject.REMOVE
                self.collision_pub.publish(remove_obj)

        self.get_logger().info('Cleared existing objects from planning scene')

        # Cancel the timer after first execution (one-shot behavior)
        self.startup_timer.cancel()
        self.startup_timer = None

    def pointcloud_callback(self, msg):
        """Store latest point cloud for processing"""
        self.latest_pc = msg

    def process_latest_pointcloud(self):
        """Process the latest point cloud to detect objects"""
        if self.latest_pc is None:
            return

        try:
            # Transform point cloud to target frame
            transformed_pc = self.transform_pointcloud(self.latest_pc)
            if transformed_pc is None:
                return

            # Convert to numpy array
            points = self.pointcloud_to_array(transformed_pc)
            if points is None or len(points) < self.min_cluster_size:
                return

            # Downsample for faster processing
            if self.downsample_factor > 1:
                points = points[::self.downsample_factor]

            # Filter by Z bounds
            points = self.filter_by_z(points)
            if len(points) < self.min_cluster_size:
                return

            # 1. Segment plane (table/support surface)
            plane_points, non_plane_points = self.segment_plane(points)

            if plane_points is not None and len(plane_points) > 100:
                self.get_logger().info(
                    f'Detected support plane with {len(plane_points)} points',
                    throttle_duration_sec=5.0
                )

            if non_plane_points is None or len(non_plane_points) < self.min_cluster_size:
                return

            # 2. Cluster objects
            clusters = self.cluster_points(non_plane_points)
            if not clusters:
                return

            self.get_logger().info(
                f'Found {len(clusters)} clusters',
                throttle_duration_sec=5.0
            )

            # 3. Detect shapes in each cluster
            detected_objects = []
            for cluster in clusters:
                shape_info = self.detect_shape(cluster)
                if shape_info:
                    detected_objects.append(shape_info)

            # 4. Publish collision objects and visualization markers
            if detected_objects:
                self.publish_collision_objects(detected_objects)
                self.publish_visualization_markers(detected_objects)

                self.get_logger().info(
                    f'Published {len(detected_objects)} collision objects',
                    throttle_duration_sec=5.0
                )

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}', throttle_duration_sec=5.0)

    def transform_pointcloud(self, pc_msg):
        """Transform point cloud to target frame using fast NumPy matrix operations"""
        try:
            # If already in target frame, return as-is
            if pc_msg.header.frame_id == self.target_frame:
                return pc_msg

            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                pc_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Extract all points at once (fast)
            points_list = []
            for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            if not points_list:
                return None

            # Convert to NumPy array for vectorized operations
            points = np.array(points_list, dtype=np.float32)

            # Extract translation and rotation from transform
            t = transform.transform.translation
            r = transform.transform.rotation

            # Convert quaternion to rotation matrix (vectorized)
            # Quaternion: [x, y, z, w]
            qx, qy, qz, qw = r.x, r.y, r.z, r.w

            # Rotation matrix from quaternion (standard formula)
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ], dtype=np.float32)

            # Apply transformation: p' = R*p + t (vectorized for all points)
            # points shape: (N, 3), R shape: (3, 3)
            transformed_points = np.dot(points, R.T) + np.array([t.x, t.y, t.z], dtype=np.float32)

            # Create new PointCloud2 message with transformed points
            from sensor_msgs.msg import PointField

            transformed_pc = PointCloud2()
            transformed_pc.header.stamp = pc_msg.header.stamp
            transformed_pc.header.frame_id = self.target_frame
            transformed_pc.height = 1
            transformed_pc.width = len(transformed_points)
            transformed_pc.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            transformed_pc.is_bigendian = False
            transformed_pc.point_step = 12
            transformed_pc.row_step = transformed_pc.point_step * len(transformed_points)
            transformed_pc.is_dense = True

            # Pack points into binary data (vectorized with tobytes)
            transformed_pc.data = transformed_points.astype(np.float32).tobytes()

            return transformed_pc

        except Exception as e:
            self.get_logger().warn(
                f'TF transform failed: {e}',
                throttle_duration_sec=5.0
            )
            return None

    def pointcloud_to_array(self, pc_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            # Extract XYZ points
            points = []
            for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if not points:
                return None

            return np.array(points)

        except Exception as e:
            self.get_logger().warn(f'Failed to convert point cloud: {e}')
            return None

    def filter_by_z(self, points):
        """Filter points by Z coordinate bounds"""
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        return points[mask]

    def segment_plane(self, points):
        """Segment largest plane using RANSAC"""
        if not SKLEARN_AVAILABLE or len(points) < 3:
            return None, points

        try:
            # Fit plane using RANSAC: z = ax + by + c
            X = points[:, :2]  # x, y
            y = points[:, 2]    # z

            ransac = RANSACRegressor(
                min_samples=3,
                residual_threshold=self.plane_threshold,
                max_trials=1000
            )
            ransac.fit(X, y)

            # Get inliers (plane points) and outliers (objects)
            inlier_mask = ransac.inlier_mask_
            plane_points = points[inlier_mask]
            non_plane_points = points[~inlier_mask]

            return plane_points, non_plane_points

        except Exception as e:
            self.get_logger().warn(f'Plane segmentation failed: {e}')
            return None, points

    def cluster_points(self, points):
        """Cluster points using DBSCAN"""
        if not SKLEARN_AVAILABLE or len(points) < self.min_cluster_size:
            # Fallback: treat all points as one cluster
            if len(points) >= self.min_cluster_size:
                return [points]
            return []

        try:
            clustering = DBSCAN(
                eps=self.cluster_tolerance,
                min_samples=10
            ).fit(points)

            labels = clustering.labels_
            unique_labels = set(labels)

            clusters = []
            for label in unique_labels:
                if label == -1:  # Noise
                    continue

                cluster_mask = (labels == label)
                cluster = points[cluster_mask]

                if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                    clusters.append(cluster)

            return clusters

        except Exception as e:
            self.get_logger().warn(f'Clustering failed: {e}')
            return []

    def detect_shape(self, cluster):
        """Detect geometric shape (cylinder or box) from point cluster with improved algorithms"""
        if len(cluster) < 10:
            return None

        # Compute bounding box
        min_coords = np.min(cluster, axis=0)
        max_coords = np.max(cluster, axis=0)
        dimensions = max_coords - min_coords
        center = (min_coords + max_coords) / 2.0

        # Check if height is sufficient
        height = dimensions[2]
        if height < self.min_height:
            return None

        # Filter out very small objects (likely noise) - minimum footprint area
        footprint_area = dimensions[0] * dimensions[1]
        if footprint_area < 0.001:  # Less than 1 square cm
            return None

        # Filter out objects that are too thin (likely edges/artifacts)
        min_dimension = min(dimensions[0], dimensions[1])
        if min_dimension < 0.02:  # Less than 2cm in any horizontal direction
            return None

        # Compute 2D projection (x, y) properties for better shape analysis
        xy_points = cluster[:, :2]
        xy_center = np.mean(xy_points, axis=0)

        # Method 1: Radial distance analysis (improved)
        distances = np.linalg.norm(xy_points - xy_center, axis=1)
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        circularity_ratio = std_distance / (mean_distance + 1e-6)

        # Method 2: Aspect ratio analysis
        width = dimensions[0]
        depth = dimensions[1]
        aspect_ratio = min(width, depth) / (max(width, depth) + 1e-6)

        # Method 3: Convex hull area vs circle area (requires scipy)
        try:
            from scipy.spatial import ConvexHull
            if len(xy_points) >= 4:
                hull = ConvexHull(xy_points)
                hull_area = hull.volume  # In 2D, volume is area
                circle_area = np.pi * (mean_distance ** 2)
                area_ratio = hull_area / (circle_area + 1e-6)
            else:
                area_ratio = 1.0
        except:
            area_ratio = 1.0

        # Method 4: Moment of inertia (shape elongation)
        centered_xy = xy_points - xy_center
        cov_matrix = np.cov(centered_xy.T)
        eigenvalues = np.linalg.eigvalsh(cov_matrix)
        elongation = np.sqrt(eigenvalues[1]) / (np.sqrt(eigenvalues[0]) + 1e-6)

        # Decision logic: Multiple criteria for better cylinder detection
        is_circular_by_stddev = circularity_ratio < 0.25  # Tighter threshold
        is_circular_by_aspect = aspect_ratio > 0.7  # Nearly square footprint
        is_circular_by_area = 0.7 < area_ratio < 1.3  # Area close to circle
        is_not_elongated = elongation < 2.0  # Not too stretched
        is_valid_radius = self.cylinder_radius_min < mean_distance < self.cylinder_radius_max

        # Cylinder score: more criteria met = more likely cylinder
        cylinder_score = sum([
            is_circular_by_stddev,
            is_circular_by_aspect,
            is_circular_by_area,
            is_not_elongated,
            is_valid_radius
        ])

        # Detect as cylinder if at least 3 out of 5 criteria are met
        if cylinder_score >= 3:
            # Refine cylinder parameters
            # Use median distance for more robust radius estimation
            radius = np.median(distances)

            # Use bottom of bounding box as base (more stable for grasping)
            position = [xy_center[0], xy_center[1], min_coords[2] + height / 2.0]

            return {
                'type': 'cylinder',
                'position': position,
                'radius': radius,
                'height': height,
                'confidence': cylinder_score / 5.0  # Normalized confidence
            }
        else:
            # Detected as box
            # Refine box dimensions to be slightly larger for collision safety
            padding = 0.01  # 1cm padding
            safe_dimensions = [
                dimensions[0] + padding,
                dimensions[1] + padding,
                dimensions[2] + padding
            ]

            return {
                'type': 'box',
                'position': [center[0], center[1], center[2]],
                'dimensions': safe_dimensions,
                'confidence': (5 - cylinder_score) / 5.0  # Inverse confidence
            }

    def publish_collision_objects(self, detected_objects):
        """Publish detected objects as MoveIt collision objects"""
        # Step 1: Remove all previously detected objects
        for old_id in self.current_object_ids:
            remove_obj = CollisionObject()
            remove_obj.header.frame_id = self.target_frame
            remove_obj.header.stamp = self.get_clock().now().to_msg()
            remove_obj.id = old_id
            remove_obj.operation = CollisionObject.REMOVE
            self.collision_pub.publish(remove_obj)

        # Clear the old ID list
        self.current_object_ids = []

        # Step 2: Add newly detected objects
        for idx, obj_info in enumerate(detected_objects):
            collision_obj = CollisionObject()
            collision_obj.header.frame_id = self.target_frame
            collision_obj.header.stamp = self.get_clock().now().to_msg()

            # Use consistent IDs based on detection order (obj_0, obj_1, ...)
            collision_obj.id = f"detected_{obj_info['type']}_{idx}"
            self.current_object_ids.append(collision_obj.id)

            # Operation: ADD
            collision_obj.operation = CollisionObject.ADD

            # Create primitive shape
            primitive = SolidPrimitive()

            if obj_info['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [float(obj_info['height']), float(obj_info['radius'])]
            else:  # box
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [float(d) for d in obj_info['dimensions']]

            collision_obj.primitives.append(primitive)

            # Pose
            pose = Pose()
            pose.position.x = float(obj_info['position'][0])
            pose.position.y = float(obj_info['position'][1])
            pose.position.z = float(obj_info['position'][2])
            pose.orientation.w = 1.0  # No rotation

            collision_obj.primitive_poses.append(pose)

            # Publish
            self.collision_pub.publish(collision_obj)

    def publish_visualization_markers(self, detected_objects):
        """Publish visualization markers for detected objects"""
        marker_array = MarkerArray()

        for idx, obj_info in enumerate(detected_objects):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detected_objects"
            marker.id = idx
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = float(obj_info['position'][0])
            marker.pose.position.y = float(obj_info['position'][1])
            marker.pose.position.z = float(obj_info['position'][2])
            marker.pose.orientation.w = 1.0

            # Color (green for cylinders, blue for boxes)
            if obj_info['type'] == 'cylinder':
                marker.type = Marker.CYLINDER
                marker.scale.x = float(obj_info['radius'] * 2)
                marker.scale.y = float(obj_info['radius'] * 2)
                marker.scale.z = float(obj_info['height'])
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)  # Green
            else:  # box
                marker.type = Marker.CUBE
                marker.scale.x = float(obj_info['dimensions'][0])
                marker.scale.y = float(obj_info['dimensions'][1])
                marker.scale.z = float(obj_info['dimensions'][2])
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)  # Blue

            # Set lifetime to 0 for persistent markers (never disappear)
            marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()

            marker_array.markers.append(marker)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
