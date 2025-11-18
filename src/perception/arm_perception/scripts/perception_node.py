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
        self.declare_parameter('cluster_tolerance', 0.02)  # 2cm for clustering
        self.declare_parameter('min_cluster_size', 50)  # Minimum points per cluster
        self.declare_parameter('max_cluster_size', 5000)  # Maximum points per cluster
        self.declare_parameter('cylinder_radius_min', 0.01)  # 1cm minimum cylinder radius
        self.declare_parameter('cylinder_radius_max', 0.15)  # 15cm maximum cylinder radius
        self.declare_parameter('min_object_height', 0.02)  # 2cm minimum object height
        self.declare_parameter('processing_rate', 1.0)  # Hz - how often to process
        self.declare_parameter('point_downsample_factor', 4)  # Downsample every Nth point for speed
        self.declare_parameter('z_min', -0.5)  # Minimum Z in target frame
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

        # QoS profile matching camera publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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

        self.get_logger().info(f'Perception node started!')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info(f'Processing rate: {self.processing_rate} Hz')

        if not SKLEARN_AVAILABLE:
            self.get_logger().warn('sklearn not available - using simplified algorithms')

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
            for idx, cluster in enumerate(clusters):
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
        """Transform point cloud to target frame"""
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

            # Workaround for tf2_sensor_msgs PointCloud2 field dtype issues:
            # Extract points, transform them manually, then recreate PointCloud2
            points_list = []
            for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
                # Transform each point using the transform
                x_in = point[0]
                y_in = point[1]
                z_in = point[2]

                # Apply translation and rotation
                t = transform.transform.translation
                r = transform.transform.rotation

                # Convert quaternion to rotation matrix (simplified for efficiency)
                # Using direct transformation instead of full matrix multiplication
                from geometry_msgs.msg import PointStamped
                pt_in = PointStamped()
                pt_in.header = pc_msg.header
                pt_in.point.x = x_in
                pt_in.point.y = y_in
                pt_in.point.z = z_in

                # Transform point
                import tf2_geometry_msgs
                pt_out = tf2_geometry_msgs.do_transform_point(pt_in, transform)

                points_list.append([pt_out.point.x, pt_out.point.y, pt_out.point.z])

            if not points_list:
                return None

            # Create new PointCloud2 message with transformed points
            import struct
            from sensor_msgs.msg import PointField

            transformed_pc = PointCloud2()
            transformed_pc.header.stamp = pc_msg.header.stamp
            transformed_pc.header.frame_id = self.target_frame
            transformed_pc.height = 1
            transformed_pc.width = len(points_list)
            transformed_pc.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            transformed_pc.is_bigendian = False
            transformed_pc.point_step = 12
            transformed_pc.row_step = transformed_pc.point_step * len(points_list)
            transformed_pc.is_dense = True

            # Pack points into binary data
            buffer = []
            for pt in points_list:
                buffer.append(struct.pack('fff', pt[0], pt[1], pt[2]))
            transformed_pc.data = b''.join(buffer)

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
        """Detect geometric shape (cylinder or box) from point cluster"""
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

        # Compute 2D projection (x, y) properties
        xy_points = cluster[:, :2]
        xy_center = np.mean(xy_points, axis=0)

        # Compute distances from centroid
        distances = np.linalg.norm(xy_points - xy_center, axis=1)
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)

        # Heuristic: if distances are consistent, likely a cylinder
        # If standard deviation is small relative to mean, it's circular
        circularity_ratio = std_distance / (mean_distance + 1e-6)

        if circularity_ratio < 0.3 and self.cylinder_radius_min < mean_distance < self.cylinder_radius_max:
            # Detected as cylinder
            return {
                'type': 'cylinder',
                'position': [center[0], center[1], center[2]],
                'radius': mean_distance,
                'height': height
            }
        else:
            # Detected as box
            return {
                'type': 'box',
                'position': [center[0], center[1], center[2]],
                'dimensions': [dimensions[0], dimensions[1], dimensions[2]]
            }

    def publish_collision_objects(self, detected_objects):
        """Publish detected objects as MoveIt collision objects"""
        for obj_info in detected_objects:
            collision_obj = CollisionObject()
            collision_obj.header.frame_id = self.target_frame
            collision_obj.header.stamp = self.get_clock().now().to_msg()

            # Unique ID
            self.object_id_counter += 1
            collision_obj.id = f"{obj_info['type']}_{self.object_id_counter}"

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
