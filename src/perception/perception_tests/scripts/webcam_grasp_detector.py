#!/usr/bin/env python3
"""
Webcam-based Object Grasp Detection (No Depth Required)
Detects objects and grasp points using only RGB webcam.
Uses assumed depth for quick testing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional
import math


class GraspPoint:
    """Represents a detected grasp point."""
    def __init__(self):
        self.pixel_location: Tuple[int, int] = (0, 0)
        self.world_location: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.orientation: float = 0.0  # radians
        self.grasp_type: str = "unknown"
        self.confidence: float = 0.0
        self.width: float = 0.0  # meters


class DetectedObject:
    """Represents a detected object."""
    def __init__(self):
        self.contour: np.ndarray = None
        self.center: Tuple[int, int] = (0, 0)
        self.center_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.orientation: float = 0.0
        self.bounding_box: cv2.RotatedRect = None
        self.area: float = 0.0
        self.grasp_points: List[GraspPoint] = []
        self.object_type: str = "unknown"


class WebcamGraspDetector(Node):
    """
    Webcam-only grasp detector for quick testing without depth camera.

    Subscribed Topics:
        - /camera/image_raw (sensor_msgs/Image): Webcam RGB image

    Published Topics:
        - /grasp_detector/visualization (sensor_msgs/Image): Annotated image
        - /grasp_detector/best_grasp (geometry_msgs/PoseStamped): Best grasp (with assumed depth)
    """

    def __init__(self):
        super().__init__('webcam_grasp_detector')

        # Parameters
        self.declare_parameter('min_object_area', 2000)
        self.declare_parameter('max_object_area', 150000)
        self.declare_parameter('assumed_depth', 0.5)  # Assume objects at 50cm
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('webcam_device', 0)  # /dev/video0
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.min_area = self.get_parameter('min_object_area').value
        self.max_area = self.get_parameter('max_object_area').value
        self.assumed_depth = self.get_parameter('assumed_depth').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.webcam_device = self.get_parameter('webcam_device').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics (approximate for typical webcam)
        self.fx = 600.0
        self.fy = 600.0
        self.cx = self.img_width / 2.0
        self.cy = self.img_height / 2.0

        # Open webcam
        self.cap = cv2.VideoCapture(self.webcam_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open webcam device {self.webcam_device}')
            return

        # Publishers
        self.vis_pub = self.create_publisher(Image, '/grasp_detector/visualization', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_detector/best_grasp', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_detector/markers', 10)

        # Timer to process frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

        self.get_logger().info('Webcam Grasp Detector initialized')
        self.get_logger().info(f'Webcam device: {self.webcam_device}')
        self.get_logger().info(f'Resolution: {self.img_width}x{self.img_height}')
        self.get_logger().info(f'Assumed depth: {self.assumed_depth}m')
        self.get_logger().info(f'Object area range: {self.min_area} - {self.max_area} pixels')

    def process_frame(self):
        """Capture and process webcam frame."""
        ret, frame = self.cap.read()
        if not ret:
            return

        # Detect objects
        objects = self.detect_objects(frame)

        if len(objects) == 0:
            # Just publish raw frame
            self.publish_visualization(frame)
            return

        # Compute grasp points
        for obj in objects:
            self.compute_grasp_points(obj)

        # Publish best grasp
        best_object = self.select_best_object(objects)
        if best_object and len(best_object.grasp_points) > 0:
            self.publish_best_grasp(best_object)

        # Visualize
        vis_image = self.visualize_detections(frame.copy(), objects)
        self.publish_visualization(vis_image)

        # Publish markers
        self.publish_markers(objects)

    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """Detect objects using contour detection."""
        objects = []

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Adaptive thresholding
        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 3)

        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_area or area > self.max_area:
                continue

            # Create detected object
            obj = DetectedObject()
            obj.contour = contour
            obj.area = area

            # Compute center
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            obj.center = (cx, cy)

            # Project to 3D (using assumed depth)
            obj.center_3d = self.project_2d_to_3d(cx, cy)

            # Fit oriented bounding box
            if len(contour) >= 5:
                obj.bounding_box = cv2.minAreaRect(contour)
                angle = obj.bounding_box[2]
                obj.orientation = np.deg2rad(angle)

            # Classify shape
            obj.object_type = self.classify_object_shape(contour, obj.bounding_box)

            objects.append(obj)

        self.get_logger().info(f'Detected {len(objects)} objects', throttle_duration_sec=1.0)
        return objects

    def classify_object_shape(self, contour: np.ndarray, bbox: cv2.RotatedRect) -> str:
        """Simple shape classification."""
        if bbox is None:
            return "unknown"

        width, height = bbox[1]
        if height == 0:
            return "unknown"

        aspect_ratio = max(width, height) / min(width, height)

        # Circularity
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
        """Compute grasp points for object."""
        grasp_points = []

        # Center grasp (always)
        center_grasp = self.compute_center_grasp(obj)
        if center_grasp:
            grasp_points.append(center_grasp)

        # Edge grasps (for elongated)
        if obj.object_type == "elongated":
            edge_grasps = self.compute_edge_grasps(obj)
            grasp_points.extend(edge_grasps)

        # Corner grasps
        if obj.object_type in ["square", "irregular"]:
            corner_grasps = self.compute_corner_grasps(obj)
            grasp_points.extend(corner_grasps)

        # Handle detection
        handle_grasps = self.detect_handles(obj)
        grasp_points.extend(handle_grasps)

        obj.grasp_points = grasp_points

    def compute_center_grasp(self, obj: DetectedObject) -> Optional[GraspPoint]:
        """Center grasp."""
        grasp = GraspPoint()
        grasp.pixel_location = obj.center
        grasp.world_location = obj.center_3d
        grasp.orientation = obj.orientation
        grasp.grasp_type = "center"
        grasp.confidence = 0.8

        if obj.bounding_box:
            width, height = obj.bounding_box[1]
            grasp.width = min(width, height) * self.get_pixel_to_meter_ratio()

        return grasp

    def compute_edge_grasps(self, obj: DetectedObject) -> List[GraspPoint]:
        """Edge grasps for elongated objects."""
        grasps = []

        if obj.bounding_box is None:
            return grasps

        width, height = obj.bounding_box[1]
        center = obj.bounding_box[0]
        angle = np.deg2rad(obj.bounding_box[2])

        if width > height:
            offset = width / 4
            for sign in [-1, 1]:
                px = int(center[0] + sign * offset * np.cos(angle))
                py = int(center[1] + sign * offset * np.sin(angle))

                world_pos = self.project_2d_to_3d(px, py)
                if world_pos:
                    grasp = GraspPoint()
                    grasp.pixel_location = (px, py)
                    grasp.world_location = world_pos
                    grasp.orientation = angle + np.pi/2
                    grasp.grasp_type = "edge"
                    grasp.confidence = 0.7
                    grasp.width = height * self.get_pixel_to_meter_ratio()
                    grasps.append(grasp)

        return grasps

    def compute_corner_grasps(self, obj: DetectedObject) -> List[GraspPoint]:
        """Corner grasps."""
        grasps = []

        if obj.bounding_box is None:
            return grasps

        box_points = cv2.boxPoints(obj.bounding_box)

        for point in box_points:
            px, py = int(point[0]), int(point[1])
            world_pos = self.project_2d_to_3d(px, py)

            if world_pos:
                grasp = GraspPoint()
                grasp.pixel_location = (px, py)
                grasp.world_location = world_pos

                dx = obj.center[0] - px
                dy = obj.center[1] - py
                grasp.orientation = np.arctan2(dy, dx)

                grasp.grasp_type = "corner"
                grasp.confidence = 0.6
                grasps.append(grasp)

        return grasps

    def detect_handles(self, obj: DetectedObject) -> List[GraspPoint]:
        """Detect handles via convexity defects."""
        grasps = []

        hull = cv2.convexHull(obj.contour, returnPoints=False)

        if len(hull) > 3 and len(obj.contour) > 3:
            defects = cv2.convexityDefects(obj.contour, hull)

            if defects is not None:
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]
                    depth = d / 256.0

                    if depth > 15:  # Significant concavity
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

    def project_2d_to_3d(self, u: int, v: int) -> Tuple[float, float, float]:
        """Project 2D to 3D using assumed depth."""
        z = self.assumed_depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def get_pixel_to_meter_ratio(self) -> float:
        """Meters per pixel at assumed depth."""
        return self.assumed_depth / self.fx

    def select_best_object(self, objects: List[DetectedObject]) -> Optional[DetectedObject]:
        """Select best object (largest by area)."""
        valid = [obj for obj in objects if len(obj.grasp_points) > 0]

        if not valid:
            return None

        # Return largest object
        return max(valid, key=lambda obj: obj.area)

    def publish_best_grasp(self, obj: DetectedObject):
        """Publish best grasp pose."""
        best_grasp = max(obj.grasp_points, key=lambda g: g.confidence)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = best_grasp.world_location[0]
        pose_msg.pose.position.y = best_grasp.world_location[1]
        pose_msg.pose.position.z = best_grasp.world_location[2]

        quat = self.angle_to_quaternion(best_grasp.orientation)
        pose_msg.pose.orientation = quat

        self.grasp_pub.publish(pose_msg)

        self.get_logger().info(
            f'Grasp: {best_grasp.grasp_type}, '
            f'pos=({best_grasp.world_location[0]:.3f}, '
            f'{best_grasp.world_location[1]:.3f}, '
            f'{best_grasp.world_location[2]:.3f}), '
            f'conf={best_grasp.confidence:.2f}',
            throttle_duration_sec=1.0)

    def angle_to_quaternion(self, angle: float) -> Quaternion:
        """Convert angle to quaternion."""
        quat = Quaternion()
        quat.w = math.cos(angle / 2.0)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(angle / 2.0)
        return quat

    def visualize_detections(self, image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
        """Draw detections on image."""
        vis = image.copy()

        for obj in objects:
            # Draw contour
            cv2.drawContours(vis, [obj.contour], -1, (0, 255, 0), 2)

            # Draw bounding box
            if obj.bounding_box:
                box = cv2.boxPoints(obj.bounding_box)
                box = np.int0(box)
                cv2.drawContours(vis, [box], 0, (255, 0, 0), 2)

            # Draw center
            cv2.circle(vis, obj.center, 5, (0, 0, 255), -1)

            # Draw grasp points
            color_map = {
                "center": (255, 255, 0),
                "edge": (255, 0, 255),
                "corner": (0, 255, 255),
                "handle": (255, 128, 0),
            }

            for grasp in obj.grasp_points:
                px, py = grasp.pixel_location
                color = color_map.get(grasp.grasp_type, (255, 255, 255))

                cv2.circle(vis, (px, py), 8, color, 2)

                # Orientation arrow
                length = 40
                end_x = int(px + length * np.cos(grasp.orientation))
                end_y = int(py + length * np.sin(grasp.orientation))
                cv2.arrowedLine(vis, (px, py), (end_x, end_y), color, 2)

                # Confidence
                cv2.putText(vis, f'{grasp.confidence:.2f}', (px + 10, py - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Object info
            info = f'{obj.object_type} ({obj.area:.0f}px)'
            cv2.putText(vis, info, (obj.center[0] - 40, obj.center[1] - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Legend
        y_offset = 30
        cv2.putText(vis, "Cyan=Center, Magenta=Edge, Yellow=Corner, Orange=Handle",
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return vis

    def publish_visualization(self, image: np.ndarray):
        """Publish visualization."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.vis_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Vis error: {e}')

    def publish_markers(self, objects: List[DetectedObject]):
        """Publish RViz markers."""
        marker_array = MarkerArray()
        marker_id = 0

        for obj in objects:
            # Object center
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

            # Grasp arrows
            for grasp in obj.grasp_points:
                marker = Marker()
                marker.header.frame_id = self.camera_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grasps"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker.pose.position.x = grasp.world_location[0]
                marker.pose.position.y = grasp.world_location[1]
                marker.pose.position.z = grasp.world_location[2]

                quat = self.angle_to_quaternion(grasp.orientation)
                marker.pose.orientation = quat

                marker.scale.x = 0.08
                marker.scale.y = 0.01
                marker.scale.z = 0.01

                marker.color.r = 1.0 - grasp.confidence
                marker.color.g = grasp.confidence
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def destroy_node(self):
        """Cleanup."""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamGraspDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
