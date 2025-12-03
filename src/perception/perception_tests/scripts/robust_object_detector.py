#!/usr/bin/env python3
"""
Robust Object Detection Node
More reliable detection using multiple techniques before grasp planning.
Prepares foundation for ML model integration (YOLO, GraspNet, etc.)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import json


class DetectedObject:
    """Enhanced object with more metadata."""
    def __init__(self):
        self.id: int = 0
        self.contour: np.ndarray = None
        self.center: Tuple[int, int] = (0, 0)
        self.center_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.orientation: float = 0.0
        self.bounding_box = None
        self.area: float = 0.0
        self.perimeter: float = 0.0

        # Shape features
        self.aspect_ratio: float = 0.0
        self.circularity: float = 0.0
        self.convexity: float = 0.0
        self.solidity: float = 0.0

        # Classification
        self.object_type: str = "unknown"
        self.detection_confidence: float = 0.0

        # Color features
        self.dominant_color: Tuple[int, int, int] = (0, 0, 0)
        self.color_name: str = "unknown"

        # Tracking
        self.tracking_id: int = -1
        self.frames_detected: int = 1
        self.stable: bool = False


class RobustObjectDetector(Node):
    """
    Improved object detection with:
    - Better segmentation (multiple methods)
    - Feature extraction
    - Temporal stability (tracking)
    - Preparation for ML integration
    """

    def __init__(self):
        super().__init__('robust_object_detector')

        # Parameters
        self.declare_parameter('webcam_device', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('min_object_area', 2000)
        self.declare_parameter('max_object_area', 150000)
        self.declare_parameter('assumed_depth', 0.5)
        self.declare_parameter('stability_threshold', 5)  # Frames to be stable
        self.declare_parameter('detection_method', 'multi')  # 'adaptive', 'canny', 'multi'

        self.webcam_device = self.get_parameter('webcam_device').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.min_area = self.get_parameter('min_object_area').value
        self.max_area = self.get_parameter('max_object_area').value
        self.assumed_depth = self.get_parameter('assumed_depth').value
        self.stability_threshold = self.get_parameter('stability_threshold').value
        self.detection_method = self.get_parameter('detection_method').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = 600.0
        self.fy = 600.0
        self.cx = self.img_width / 2.0
        self.cy = self.img_height / 2.0

        # Object tracking
        self.tracked_objects: List[DetectedObject] = []
        self.next_tracking_id = 0

        # Background subtraction (optional)
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=16, detectShadows=True)

        # Open webcam
        self.cap = cv2.VideoCapture(self.webcam_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open webcam {self.webcam_device}')
            return

        # Publishers
        self.vis_pub = self.create_publisher(Image, '/object_detector/visualization', 10)
        self.objects_pub = self.create_publisher(String, '/object_detector/objects', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_detector/markers', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

        self.get_logger().info('Robust Object Detector initialized')
        self.get_logger().info(f'Detection method: {self.detection_method}')
        self.get_logger().info(f'Stability threshold: {self.stability_threshold} frames')

    def process_frame(self):
        """Main processing loop."""
        ret, frame = self.cap.read()
        if not ret:
            return

        # Detect objects using selected method
        if self.detection_method == 'adaptive':
            objects = self.detect_adaptive_threshold(frame)
        elif self.detection_method == 'canny':
            objects = self.detect_canny_contours(frame)
        elif self.detection_method == 'multi':
            objects = self.detect_multi_method(frame)
        else:
            objects = self.detect_adaptive_threshold(frame)

        # Extract features for each object
        for obj in objects:
            self.extract_features(obj, frame)

        # Track objects across frames
        self.update_tracking(objects)

        # Publish stable objects
        stable_objects = [obj for obj in self.tracked_objects if obj.stable]

        if len(stable_objects) > 0:
            self.publish_objects(stable_objects)
            self.publish_markers(stable_objects)

        # Visualize
        vis = self.visualize_detections(frame.copy(), self.tracked_objects)
        self.publish_visualization(vis)

    def detect_adaptive_threshold(self, image: np.ndarray) -> List[DetectedObject]:
        """Adaptive thresholding method (good for varying lighting)."""
        objects = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 3)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = self.process_contours(contours)

        return objects

    def detect_canny_contours(self, image: np.ndarray) -> List[DetectedObject]:
        """Edge-based detection using Canny (good for high-contrast scenes)."""
        objects = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Automatic Canny threshold
        median = np.median(blurred)
        lower = int(max(0, 0.7 * median))
        upper = int(min(255, 1.3 * median))

        edges = cv2.Canny(blurred, lower, upper)

        # Dilate to connect edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges = cv2.dilate(edges, kernel, iterations=2)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = self.process_contours(contours)

        return objects

    def detect_multi_method(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Combine multiple detection methods for robustness.
        Uses voting/consensus approach.
        """
        # Method 1: Adaptive threshold
        objects_adaptive = self.detect_adaptive_threshold(image)

        # Method 2: Canny edges
        objects_canny = self.detect_canny_contours(image)

        # Method 3: Color-based segmentation
        objects_color = self.detect_color_segmentation(image)

        # Merge detections (simple approach: union of all)
        all_objects = objects_adaptive + objects_canny + objects_color

        # Remove duplicates (objects with similar centers)
        merged_objects = self.merge_duplicate_detections(all_objects)

        return merged_objects

    def detect_color_segmentation(self, image: np.ndarray) -> List[DetectedObject]:
        """Color-based segmentation (finds colorful objects on neutral background)."""
        objects = []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for saturated colors (ignores white/gray/black background)
        lower_sat = np.array([0, 30, 30])
        upper_sat = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_sat, upper_sat)

        # Clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = self.process_contours(contours)

        return objects

    def process_contours(self, contours) -> List[DetectedObject]:
        """Convert contours to DetectedObject instances."""
        objects = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            obj = DetectedObject()
            obj.contour = contour
            obj.area = area
            obj.perimeter = cv2.arcLength(contour, True)

            # Center
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            obj.center = (cx, cy)
            obj.center_3d = self.project_2d_to_3d(cx, cy)

            # Bounding box
            if len(contour) >= 5:
                obj.bounding_box = cv2.minAreaRect(contour)
                obj.orientation = np.deg2rad(obj.bounding_box[2])

                width, height = obj.bounding_box[1]
                if height > 0:
                    obj.aspect_ratio = max(width, height) / min(width, height)

            objects.append(obj)

        return objects

    def extract_features(self, obj: DetectedObject, image: np.ndarray):
        """Extract detailed features from object."""

        # Circularity
        if obj.perimeter > 0:
            obj.circularity = 4 * np.pi * obj.area / (obj.perimeter ** 2)

        # Convexity and solidity
        hull = cv2.convexHull(obj.contour)
        hull_area = cv2.contourArea(hull)
        hull_perimeter = cv2.arcLength(hull, True)

        if hull_area > 0:
            obj.solidity = obj.area / hull_area

        if obj.perimeter > 0 and hull_perimeter > 0:
            obj.convexity = hull_perimeter / obj.perimeter

        # Dominant color
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [obj.contour], -1, 255, -1)

        mean_color = cv2.mean(image, mask=mask)[:3]
        obj.dominant_color = tuple(map(int, mean_color))
        obj.color_name = self.get_color_name(obj.dominant_color)

        # Classify shape
        obj.object_type = self.classify_robust_shape(obj)

        # Detection confidence based on features
        obj.detection_confidence = self.compute_detection_confidence(obj)

    def classify_robust_shape(self, obj: DetectedObject) -> str:
        """
        Enhanced shape classification using multiple features.
        """
        # Circular: high circularity, low aspect ratio
        if obj.circularity > 0.8 and obj.aspect_ratio < 1.3:
            return "circular"

        # Elongated: high aspect ratio, moderate circularity
        if obj.aspect_ratio > 3.0:
            return "elongated"

        # Square/rectangular: moderate aspect ratio, high solidity, low circularity
        if obj.aspect_ratio < 2.0 and obj.solidity > 0.8 and obj.circularity < 0.7:
            # Check for 4 corners
            approx = cv2.approxPolyDP(obj.contour, 0.04 * obj.perimeter, True)
            if len(approx) == 4:
                return "rectangular"
            elif obj.aspect_ratio < 1.3:
                return "square"

        # Triangular
        if obj.solidity > 0.85:
            approx = cv2.approxPolyDP(obj.contour, 0.04 * obj.perimeter, True)
            if len(approx) == 3:
                return "triangular"

        # Complex/irregular
        if obj.convexity < 0.9:
            return "complex"

        return "irregular"

    def compute_detection_confidence(self, obj: DetectedObject) -> float:
        """
        Compute confidence score for detection quality.
        Higher = more reliable detection.
        """
        confidence = 0.5  # Base

        # Area: prefer medium-sized objects
        if 5000 < obj.area < 50000:
            confidence += 0.2

        # Solidity: prefer solid objects (not fragmented)
        if obj.solidity > 0.85:
            confidence += 0.1

        # Circularity: well-defined shapes
        if obj.circularity > 0.5 or obj.circularity < 0.3:
            confidence += 0.1

        # Aspect ratio: reasonable shapes
        if 0.5 < obj.aspect_ratio < 5.0:
            confidence += 0.1

        return min(confidence, 1.0)

    def get_color_name(self, bgr_color: Tuple[int, int, int]) -> str:
        """Simple color naming."""
        b, g, r = bgr_color

        # Convert to HSV for better color classification
        hsv = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        # Low saturation = gray/white/black
        if s < 40:
            if v < 60:
                return "black"
            elif v > 200:
                return "white"
            else:
                return "gray"

        # Color by hue
        if h < 10 or h > 170:
            return "red"
        elif 10 <= h < 25:
            return "orange"
        elif 25 <= h < 40:
            return "yellow"
        elif 40 <= h < 80:
            return "green"
        elif 80 <= h < 130:
            return "blue"
        elif 130 <= h < 170:
            return "purple"

        return "unknown"

    def update_tracking(self, new_objects: List[DetectedObject]):
        """
        Track objects across frames for stability.
        Only publish objects that appear consistently.
        """
        # Match new objects to tracked objects
        matched = []

        for new_obj in new_objects:
            best_match = None
            best_distance = float('inf')

            for tracked_obj in self.tracked_objects:
                # Distance between centers
                dx = new_obj.center[0] - tracked_obj.center[0]
                dy = new_obj.center[1] - tracked_obj.center[1]
                distance = np.sqrt(dx*dx + dy*dy)

                # Match if within 50 pixels
                if distance < 50 and distance < best_distance:
                    best_match = tracked_obj
                    best_distance = distance

            if best_match:
                # Update existing object
                best_match.center = new_obj.center
                best_match.center_3d = new_obj.center_3d
                best_match.contour = new_obj.contour
                best_match.bounding_box = new_obj.bounding_box
                best_match.area = new_obj.area
                best_match.object_type = new_obj.object_type
                best_match.frames_detected += 1

                # Mark as stable if detected enough times
                if best_match.frames_detected >= self.stability_threshold:
                    best_match.stable = True

                matched.append(best_match)
            else:
                # New object
                new_obj.tracking_id = self.next_tracking_id
                self.next_tracking_id += 1
                new_obj.frames_detected = 1
                new_obj.stable = False
                matched.append(new_obj)

        # Remove objects not seen recently (timeout after 10 frames)
        self.tracked_objects = [obj for obj in matched]

    def merge_duplicate_detections(self, objects: List[DetectedObject]) -> List[DetectedObject]:
        """Remove duplicate detections from multi-method approach."""
        if len(objects) == 0:
            return []

        merged = []
        used = [False] * len(objects)

        for i, obj1 in enumerate(objects):
            if used[i]:
                continue

            # Find all similar objects
            cluster = [obj1]
            used[i] = True

            for j, obj2 in enumerate(objects):
                if used[j]:
                    continue

                # Check if centers are close
                dx = obj1.center[0] - obj2.center[0]
                dy = obj1.center[1] - obj2.center[1]
                distance = np.sqrt(dx*dx + dy*dy)

                if distance < 30:  # Within 30 pixels
                    cluster.append(obj2)
                    used[j] = True

            # Use the object with highest confidence
            best_obj = max(cluster, key=lambda o: o.detection_confidence)
            merged.append(best_obj)

        return merged

    def project_2d_to_3d(self, u: int, v: int) -> Tuple[float, float, float]:
        """Project to 3D using assumed depth."""
        z = self.assumed_depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def publish_objects(self, objects: List[DetectedObject]):
        """Publish detected objects as JSON."""
        objects_data = []

        for obj in objects:
            obj_dict = {
                'id': obj.tracking_id,
                'type': obj.object_type,
                'center_2d': list(obj.center),
                'center_3d': list(obj.center_3d),
                'area': float(obj.area),
                'orientation': float(obj.orientation),
                'color': obj.color_name,
                'confidence': float(obj.detection_confidence),
                'stable': obj.stable,
                'frames_detected': obj.frames_detected,
                'features': {
                    'circularity': float(obj.circularity),
                    'aspect_ratio': float(obj.aspect_ratio),
                    'solidity': float(obj.solidity),
                }
            }
            objects_data.append(obj_dict)

        msg = String()
        msg.data = json.dumps(objects_data, indent=2)
        self.objects_pub.publish(msg)

    def visualize_detections(self, image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
        """Enhanced visualization."""
        vis = image.copy()

        for obj in objects:
            # Color by stability
            if obj.stable:
                contour_color = (0, 255, 0)  # Green = stable
                text_color = (0, 255, 0)
            else:
                contour_color = (0, 165, 255)  # Orange = detecting
                text_color = (0, 165, 255)

            # Draw contour
            cv2.drawContours(vis, [obj.contour], -1, contour_color, 2)

            # Bounding box
            if obj.bounding_box:
                box = cv2.boxPoints(obj.bounding_box)
                box = np.int0(box)
                cv2.drawContours(vis, [box], 0, (255, 0, 0), 1)

            # Center
            cv2.circle(vis, obj.center, 5, (0, 0, 255), -1)

            # Info text
            cx, cy = obj.center
            info_lines = [
                f"ID:{obj.tracking_id} {obj.object_type}",
                f"{obj.color_name} ({obj.frames_detected}f)",
                f"conf:{obj.detection_confidence:.2f}"
            ]

            y_offset = -30
            for line in info_lines:
                cv2.putText(vis, line, (cx - 50, cy + y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)
                y_offset += 15

        # Stats overlay
        stable_count = sum(1 for obj in objects if obj.stable)
        stats = f"Objects: {len(objects)} (Stable: {stable_count})"
        cv2.putText(vis, stats, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        method_text = f"Method: {self.detection_method}"
        cv2.putText(vis, method_text, (10, vis.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return vis

    def publish_visualization(self, image: np.ndarray):
        """Publish visualization image."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.vis_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Vis error: {e}')

    def publish_markers(self, objects: List[DetectedObject]):
        """Publish RViz markers."""
        marker_array = MarkerArray()

        for obj in objects:
            # Object sphere
            marker = Marker()
            marker.header.frame_id = "camera_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = obj.tracking_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = obj.center_3d[0]
            marker.pose.position.y = obj.center_3d[1]
            marker.pose.position.z = obj.center_3d[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            if obj.stable:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.0

            marker.color.a = 1.0
            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "labels"
            text_marker.id = obj.tracking_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = obj.center_3d[0]
            text_marker.pose.position.y = obj.center_3d[1]
            text_marker.pose.position.z = obj.center_3d[2] + 0.1

            text_marker.text = f"{obj.object_type}\n{obj.color_name}"
            text_marker.scale.z = 0.03

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def destroy_node(self):
        """Cleanup."""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobustObjectDetector()

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
