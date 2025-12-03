#!/usr/bin/env python3
"""
OpenCV-based Object Grasp Detection Node
Detects objects, their position, orientation, and optimal grasp points using RGB-D data.
Single file implementation for quick and easy object grasping.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import math


class GraspPoint:
    """Represents a detected grasp point with position, orientation, and metadata."""
    def __init__(self):
        self.pixel_location: Tuple[int, int] = (0, 0)  # (x, y) in image
        self.world_location: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (x, y, z) in meters
        self.orientation: float = 0.0  # Grasp angle in radians
        self.grasp_type: str = "unknown"  # "center", "edge", "handle", "corner"
        self.confidence: float = 0.0  # 0.0 to 1.0
        self.width: float = 0.0  # Estimated object width at grasp point (meters)


class DetectedObject:
    """Represents a detected object with its properties."""
    def __init__(self):
        self.contour: np.ndarray = None
        self.center: Tuple[int, int] = (0, 0)
        self.center_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.orientation: float = 0.0  # Principal axis angle in radians
        self.bounding_box: cv2.RotatedRect = None
        self.area: float = 0.0
        self.grasp_points: List[GraspPoint] = []
        self.object_type: str = "unknown"


class OpenCVGraspDetector(Node):
    """
    ROS 2 node for detecting objects and their optimal grasp points using OpenCV.

    Subscribed Topics:
        - /camera/color/image_raw (sensor_msgs/Image): RGB image
        - /camera/depth/image_raw (sensor_msgs/Image): Depth image (aligned)
        - /camera/color/camera_info (sensor_msgs/CameraInfo): Camera intrinsics

    Published Topics:
        - /grasp_detector/visualization (sensor_msgs/Image): Annotated detection image
        - /grasp_detector/best_grasp (geometry_msgs/PoseStamped): Best grasp pose
        - /grasp_detector/markers (visualization_msgs/MarkerArray): RViz markers
    """

    def __init__(self):
        super().__init__('opencv_grasp_detector')

        # Parameters
        self.declare_parameter('min_object_area', 1000)  # pixels
        self.declare_parameter('max_object_area', 100000)  # pixels
        self.declare_parameter('depth_min', 0.3)  # meters
        self.declare_parameter('depth_max', 2.0)  # meters
        self.declare_parameter('visualize', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        self.min_area = self.get_parameter('min_object_area').value
        self.max_area = self.get_parameter('max_object_area').value
        self.depth_min = self.get_parameter('depth_min').value
        self.depth_max = self.get_parameter('depth_max').value
        self.visualize = self.get_parameter('visualize').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics (will be updated from CameraInfo)
        self.fx = 615.0  # Default focal length
        self.fy = 615.0
        self.cx = 320.0  # Default principal point
        self.cy = 240.0

        # Cached images
        self.rgb_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        # Publishers
        self.vis_pub = self.create_publisher(Image, '/grasp_detector/visualization', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_detector/best_grasp', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_detector/markers', 10)

        self.get_logger().info('OpenCV Grasp Detector initialized')
        self.get_logger().info(f'Looking for objects between {self.min_area} and {self.max_area} pixels')
        self.get_logger().info(f'Depth range: {self.depth_min}m to {self.depth_max}m')

    def info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from CameraInfo."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_callback(self, msg: Image):
        """Process incoming RGB image."""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def depth_callback(self, msg: Image):
        """Process incoming depth image."""
        try:
            # Depth image in millimeters (uint16) or meters (float32)
            if msg.encoding == '16UC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.depth_image = self.depth_image.astype(np.float32) / 1000.0  # Convert to meters
            elif msg.encoding == '32FC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unexpected depth encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def process_frame(self):
        """Main processing pipeline."""
        if self.rgb_image is None or self.depth_image is None:
            return

        # Detect objects
        objects = self.detect_objects(self.rgb_image)

        if len(objects) == 0:
            return

        # Find grasp points for each object
        for obj in objects:
            self.compute_grasp_points(obj)

        # Publish best grasp
        best_object = self.select_best_object(objects)
        if best_object and len(best_object.grasp_points) > 0:
            self.publish_best_grasp(best_object)

        # Visualize
        if self.visualize:
            vis_image = self.visualize_detections(self.rgb_image.copy(), objects)
            self.publish_visualization(vis_image)

        # Publish RViz markers
        self.publish_markers(objects)

    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Detect objects in the RGB image using contour detection.

        Strategy:
        1. Adaptive thresholding for robustness to lighting
        2. Morphological operations to clean noise
        3. Contour detection and filtering by area
        4. Fit oriented bounding boxes
        """
        objects = []

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Adaptive thresholding (works better than fixed threshold)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 11, 2)

        # Morphological operations to clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter by area
            if area < self.min_area or area > self.max_area:
                continue

            # Create detected object
            obj = DetectedObject()
            obj.contour = contour
            obj.area = area

            # Compute moments for center
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            obj.center = (cx, cy)

            # Get 3D center
            obj.center_3d = self.project_2d_to_3d(cx, cy)
            if obj.center_3d is None:
                continue

            # Fit oriented bounding box
            if len(contour) >= 5:
                obj.bounding_box = cv2.minAreaRect(contour)

                # Compute principal axis orientation
                angle = obj.bounding_box[2]
                obj.orientation = np.deg2rad(angle)

            # Classify object type (simple heuristic)
            obj.object_type = self.classify_object_shape(contour, obj.bounding_box)

            objects.append(obj)

        self.get_logger().info(f'Detected {len(objects)} objects')
        return objects

    def classify_object_shape(self, contour: np.ndarray, bbox: cv2.RotatedRect) -> str:
        """Simple shape classification based on geometric properties."""
        if bbox is None:
            return "unknown"

        # Compute aspect ratio
        width, height = bbox[1]
        if height == 0:
            return "unknown"

        aspect_ratio = max(width, height) / min(width, height)

        # Compute circularity
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return "unknown"

        circularity = 4 * np.pi * area / (perimeter ** 2)

        # Classify
        if circularity > 0.8:
            return "circular"
        elif aspect_ratio > 3.0:
            return "elongated"
        elif aspect_ratio < 1.5 and circularity > 0.6:
            return "square"
        else:
            return "irregular"

    def compute_grasp_points(self, obj: DetectedObject):
        """
        Compute optimal grasp points for an object.

        Strategies:
        1. Center grasp - simplest, works for symmetric objects
        2. Edge grasp - for flat objects, grasp along longest edge
        3. Corner grasp - for rectangular objects
        4. Handle detection - for objects with handles
        """
        grasp_points = []

        # Strategy 1: Center grasp (always available)
        center_grasp = self.compute_center_grasp(obj)
        if center_grasp:
            grasp_points.append(center_grasp)

        # Strategy 2: Edge grasp (for elongated objects)
        if obj.object_type == "elongated":
            edge_grasps = self.compute_edge_grasps(obj)
            grasp_points.extend(edge_grasps)

        # Strategy 3: Corner grasps (for rectangular objects)
        if obj.object_type in ["square", "irregular"]:
            corner_grasps = self.compute_corner_grasps(obj)
            grasp_points.extend(corner_grasps)

        # Strategy 4: Handle detection (using contour analysis)
        handle_grasps = self.detect_handles(obj)
        grasp_points.extend(handle_grasps)

        obj.grasp_points = grasp_points

    def compute_center_grasp(self, obj: DetectedObject) -> Optional[GraspPoint]:
        """Compute grasp at object center - simplest and most robust."""
        cx, cy = obj.center

        grasp = GraspPoint()
        grasp.pixel_location = (cx, cy)
        grasp.world_location = obj.center_3d
        grasp.orientation = obj.orientation
        grasp.grasp_type = "center"
        grasp.confidence = 0.8  # High confidence for center grasp

        # Estimate grasp width from bounding box
        if obj.bounding_box:
            width, height = obj.bounding_box[1]
            grasp.width = min(width, height) * self.get_pixel_to_meter_ratio(obj.center_3d[2])

        return grasp

    def compute_edge_grasps(self, obj: DetectedObject) -> List[GraspPoint]:
        """Compute grasps along the long edges of elongated objects."""
        grasps = []

        if obj.bounding_box is None:
            return grasps

        # Get the four corners of the oriented bounding box
        box_points = cv2.boxPoints(obj.bounding_box)
        box_points = np.int0(box_points)

        # Find the two longest edges (opposite sides)
        width, height = obj.bounding_box[1]
        center = obj.bounding_box[0]
        angle = np.deg2rad(obj.bounding_box[2])

        # Midpoints of long edges
        if width > height:
            # Horizontal long axis
            offset = width / 4
            for sign in [-1, 1]:
                px = int(center[0] + sign * offset * np.cos(angle))
                py = int(center[1] + sign * offset * np.sin(angle))

                world_pos = self.project_2d_to_3d(px, py)
                if world_pos:
                    grasp = GraspPoint()
                    grasp.pixel_location = (px, py)
                    grasp.world_location = world_pos
                    grasp.orientation = angle + np.pi/2  # Perpendicular to edge
                    grasp.grasp_type = "edge"
                    grasp.confidence = 0.7
                    grasp.width = height * self.get_pixel_to_meter_ratio(world_pos[2])
                    grasps.append(grasp)

        return grasps

    def compute_corner_grasps(self, obj: DetectedObject) -> List[GraspPoint]:
        """Compute grasps at object corners."""
        grasps = []

        if obj.bounding_box is None:
            return grasps

        box_points = cv2.boxPoints(obj.bounding_box)

        # Grasp at each corner
        for point in box_points:
            px, py = int(point[0]), int(point[1])

            world_pos = self.project_2d_to_3d(px, py)
            if world_pos:
                grasp = GraspPoint()
                grasp.pixel_location = (px, py)
                grasp.world_location = world_pos

                # Orientation towards center
                dx = obj.center[0] - px
                dy = obj.center[1] - py
                grasp.orientation = np.arctan2(dy, dx)

                grasp.grasp_type = "corner"
                grasp.confidence = 0.6
                grasps.append(grasp)

        return grasps

    def detect_handles(self, obj: DetectedObject) -> List[GraspPoint]:
        """
        Detect handle-like structures using contour analysis.
        Handles typically appear as concave regions or loops.
        """
        grasps = []

        # Find convexity defects (concave regions)
        hull = cv2.convexHull(obj.contour, returnPoints=False)

        if len(hull) > 3 and len(obj.contour) > 3:
            defects = cv2.convexityDefects(obj.contour, hull)

            if defects is not None:
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]

                    # Depth of defect (concavity)
                    depth = d / 256.0

                    # Significant concavity might indicate a handle
                    if depth > 10:
                        far = tuple(obj.contour[f][0])

                        world_pos = self.project_2d_to_3d(far[0], far[1])
                        if world_pos:
                            grasp = GraspPoint()
                            grasp.pixel_location = far
                            grasp.world_location = world_pos
                            grasp.orientation = obj.orientation
                            grasp.grasp_type = "handle"
                            grasp.confidence = 0.5 + min(depth / 50.0, 0.4)
                            grasps.append(grasp)

        return grasps

    def project_2d_to_3d(self, u: int, v: int) -> Optional[Tuple[float, float, float]]:
        """Project 2D pixel coordinates to 3D using depth image."""
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        # Get depth value
        z = self.depth_image[v, u]

        # Check valid depth
        if np.isnan(z) or z < self.depth_min or z > self.depth_max:
            return None

        # Project to 3D using pinhole camera model
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        return (x, y, z)

    def get_pixel_to_meter_ratio(self, depth: float) -> float:
        """Get the approximate meters per pixel at a given depth."""
        if depth <= 0:
            return 0.001
        return depth / self.fx

    def select_best_object(self, objects: List[DetectedObject]) -> Optional[DetectedObject]:
        """Select the best object to grasp (closest with valid grasp points)."""
        valid_objects = [obj for obj in objects if len(obj.grasp_points) > 0]

        if len(valid_objects) == 0:
            return None

        # Sort by distance (closest first)
        valid_objects.sort(key=lambda obj: obj.center_3d[2])

        return valid_objects[0]

    def publish_best_grasp(self, obj: DetectedObject):
        """Publish the best grasp pose for the object."""
        # Select best grasp point (highest confidence)
        best_grasp = max(obj.grasp_points, key=lambda g: g.confidence)

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = best_grasp.world_location[0]
        pose_msg.pose.position.y = best_grasp.world_location[1]
        pose_msg.pose.position.z = best_grasp.world_location[2]

        # Convert orientation angle to quaternion
        # Grasp approach from above (Z-axis down), rotate around Z by grasp angle
        quat = self.angle_to_quaternion(best_grasp.orientation)
        pose_msg.pose.orientation = quat

        self.grasp_pub.publish(pose_msg)

        self.get_logger().info(
            f'Best grasp: type={best_grasp.grasp_type}, '
            f'pos=({best_grasp.world_location[0]:.3f}, '
            f'{best_grasp.world_location[1]:.3f}, '
            f'{best_grasp.world_location[2]:.3f}), '
            f'confidence={best_grasp.confidence:.2f}')

    def angle_to_quaternion(self, angle: float) -> Quaternion:
        """Convert a rotation angle (around Z-axis) to quaternion."""
        # For grasp: approach from above, rotate around Z
        quat = Quaternion()
        quat.w = math.cos(angle / 2.0)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(angle / 2.0)
        return quat

    def visualize_detections(self, image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
        """Visualize detected objects and grasp points."""
        vis = image.copy()

        for obj in objects:
            # Draw contour
            cv2.drawContours(vis, [obj.contour], -1, (0, 255, 0), 2)

            # Draw bounding box
            if obj.bounding_box:
                box_points = cv2.boxPoints(obj.bounding_box)
                box_points = np.int0(box_points)
                cv2.drawContours(vis, [box_points], 0, (255, 0, 0), 2)

            # Draw center
            cv2.circle(vis, obj.center, 5, (0, 0, 255), -1)

            # Draw grasp points
            for grasp in obj.grasp_points:
                px, py = grasp.pixel_location

                # Color by type
                color_map = {
                    "center": (255, 255, 0),    # Cyan
                    "edge": (255, 0, 255),      # Magenta
                    "corner": (0, 255, 255),    # Yellow
                    "handle": (255, 128, 0),    # Orange
                }
                color = color_map.get(grasp.grasp_type, (255, 255, 255))

                # Draw grasp point
                cv2.circle(vis, (px, py), 8, color, 2)

                # Draw orientation
                length = 30
                end_x = int(px + length * np.cos(grasp.orientation))
                end_y = int(py + length * np.sin(grasp.orientation))
                cv2.arrowedLine(vis, (px, py), (end_x, end_y), color, 2)

                # Draw confidence
                conf_text = f'{grasp.confidence:.2f}'
                cv2.putText(vis, conf_text, (px + 10, py - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Draw object info
            info_text = f'{obj.object_type} Z={obj.center_3d[2]:.2f}m'
            cv2.putText(vis, info_text, (obj.center[0] - 30, obj.center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return vis

    def publish_visualization(self, image: np.ndarray):
        """Publish visualization image."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.vis_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Visualization publish error: {e}')

    def publish_markers(self, objects: List[DetectedObject]):
        """Publish RViz markers for 3D visualization."""
        marker_array = MarkerArray()
        marker_id = 0

        for obj_idx, obj in enumerate(objects):
            # Object center marker
            marker = Marker()
            marker.header.frame_id = self.camera_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = obj.center_3d[0]
            marker.pose.position.y = obj.center_3d[1]
            marker.pose.position.z = obj.center_3d[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

            # Grasp point markers
            for grasp in obj.grasp_points:
                marker = Marker()
                marker.header.frame_id = self.camera_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grasp_points"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker.pose.position.x = grasp.world_location[0]
                marker.pose.position.y = grasp.world_location[1]
                marker.pose.position.z = grasp.world_location[2]

                quat = self.angle_to_quaternion(grasp.orientation)
                marker.pose.orientation = quat

                marker.scale.x = 0.1  # Arrow length
                marker.scale.y = 0.01  # Arrow width
                marker.scale.z = 0.01  # Arrow height

                # Color by confidence
                marker.color.r = 1.0 - grasp.confidence
                marker.color.g = grasp.confidence
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVGraspDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
