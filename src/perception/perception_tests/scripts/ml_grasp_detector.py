#!/usr/bin/env python3
"""
ML-Based Grasp Detection Framework
Prepares for integration with learned models (GraspNet, Contact-GraspNet, etc.)
Currently uses placeholder - ready for real model integration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import json
import math


class GraspPrediction:
    """Represents a predicted grasp from ML model."""
    def __init__(self):
        self.position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 3D position (m)
        self.orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # Quaternion
        self.width: float = 0.0  # Gripper opening width (m)
        self.quality: float = 0.0  # Grasp quality score (0-1)
        self.approach_vector: Tuple[float, float, float] = (0.0, 0.0, -1.0)  # Approach direction
        self.grasp_type: str = "parallel_jaw"  # "parallel_jaw", "suction", "pinch"


class MLGraspDetector(Node):
    """
    Framework for ML-based grasp detection.

    Currently implements:
    - Basic object detection (using robust detector)
    - Placeholder grasp prediction (to be replaced with GraspNet/Contact-GraspNet)
    - Full ROS integration

    To integrate real model:
    1. Install model: pip install graspnetAPI torch
    2. Download weights: wget https://graspnet.net/models/graspnet_baseline.pth
    3. Replace predict_grasps_placeholder() with real model inference
    """

    def __init__(self):
        super().__init__('ml_grasp_detector')

        # Parameters
        self.declare_parameter('webcam_device', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('assumed_depth', 0.5)
        self.declare_parameter('model_type', 'placeholder')  # 'graspnet', 'contact_graspnet', 'placeholder'
        self.declare_parameter('model_path', '')
        self.declare_parameter('min_grasp_quality', 0.5)
        self.declare_parameter('max_grasps', 10)

        self.webcam_device = self.get_parameter('webcam_device').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.assumed_depth = self.get_parameter('assumed_depth').value
        self.model_type = self.get_parameter('model_type').value
        self.model_path = self.get_parameter('model_path').value
        self.min_quality = self.get_parameter('min_grasp_quality').value
        self.max_grasps = self.get_parameter('max_grasps').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = 600.0
        self.fy = 600.0
        self.cx = self.img_width / 2.0
        self.cy = self.img_height / 2.0

        # Load ML model
        self.model = self.load_model()

        # Open webcam
        self.cap = cv2.VideoCapture(self.webcam_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open webcam {self.webcam_device}')
            return

        # Cached depth (simulated for webcam)
        self.depth_image = None

        # Publishers
        self.vis_pub = self.create_publisher(Image, '/ml_grasp/visualization', 10)
        self.grasps_pub = self.create_publisher(String, '/ml_grasp/predictions', 10)
        self.best_grasp_pub = self.create_publisher(PoseStamped, '/ml_grasp/best_grasp', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ml_grasp/markers', 10)

        # Timer
        self.timer = self.create_timer(0.2, self.process_frame)  # 5 Hz (ML inference slower)

        self.get_logger().info('ML Grasp Detector initialized')
        self.get_logger().info(f'Model type: {self.model_type}')
        self.get_logger().info(f'Min grasp quality: {self.min_quality}')
        self.get_logger().warn('Using PLACEHOLDER model - integrate real GraspNet for production!')

    def load_model(self):
        """Load ML model (placeholder for now)."""
        if self.model_type == 'graspnet':
            return self.load_graspnet_model()
        elif self.model_type == 'contact_graspnet':
            return self.load_contact_graspnet_model()
        else:
            self.get_logger().info('Using placeholder model (geometric heuristics)')
            return None

    def load_graspnet_model(self):
        """
        Load GraspNet model (to be implemented).

        Installation:
            pip install graspnetAPI torch torchvision
            wget https://graspnet.net/models/checkpoint.tar

        Usage:
            from graspnetAPI import GraspNet
            model = GraspNet.load_model('checkpoint.tar')
            return model
        """
        self.get_logger().error('GraspNet not implemented yet - using placeholder')
        return None

    def load_contact_graspnet_model(self):
        """
        Load Contact-GraspNet model (to be implemented).

        Installation:
            git clone https://github.com/NVlabs/contact_graspnet
            pip install -r requirements.txt

        Usage:
            from contact_graspnet import ContactGraspNet
            model = ContactGraspNet(checkpoint_dir='checkpoints')
            return model
        """
        self.get_logger().error('Contact-GraspNet not implemented yet - using placeholder')
        return None

    def process_frame(self):
        """Main processing loop."""
        ret, frame = self.cap.read()
        if not ret:
            return

        # Generate simulated depth for webcam
        self.depth_image = self.generate_simulated_depth(frame)

        # Detect objects first (basic segmentation)
        objects = self.detect_objects(frame)

        if len(objects) == 0:
            self.publish_visualization(frame)
            return

        # Predict grasps for each object
        all_grasps = []
        for obj_mask, obj_center in objects:
            grasps = self.predict_grasps(frame, self.depth_image, obj_mask, obj_center)
            all_grasps.extend(grasps)

        # Filter by quality
        good_grasps = [g for g in all_grasps if g.quality >= self.min_quality]

        # Sort by quality
        good_grasps.sort(key=lambda g: g.quality, reverse=True)

        # Limit number
        top_grasps = good_grasps[:self.max_grasps]

        if len(top_grasps) > 0:
            self.publish_grasps(top_grasps)
            self.publish_best_grasp(top_grasps[0])
            self.publish_markers(top_grasps)

        # Visualize
        vis = self.visualize_grasps(frame.copy(), top_grasps, objects)
        self.publish_visualization(vis)

        self.get_logger().info(
            f'Detected {len(objects)} objects, predicted {len(top_grasps)} grasps',
            throttle_duration_sec=1.0)

    def detect_objects(self, image: np.ndarray) -> List[Tuple[np.ndarray, Tuple[int, int]]]:
        """
        Simple object detection for ML grasp prediction.
        Returns list of (mask, center) tuples.
        """
        objects = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 3)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 2000 or area > 150000:
                continue

            # Create mask for this object
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)

            # Get center
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            objects.append((mask, (cx, cy)))

        return objects

    def generate_simulated_depth(self, image: np.ndarray) -> np.ndarray:
        """
        Generate simulated depth map for webcam.
        Real depth camera would provide this directly.
        """
        h, w = image.shape[:2]

        # Simple gradient: closer at center, farther at edges
        y_grid, x_grid = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')

        cx, cy = w / 2, h / 2
        dist_from_center = np.sqrt((x_grid - cx)**2 + (y_grid - cy)**2)

        # Normalize to 0-1
        max_dist = np.sqrt(cx**2 + cy**2)
        normalized_dist = dist_from_center / max_dist

        # Map to depth range (0.3m to 0.7m)
        depth = 0.3 + normalized_dist * 0.4

        # Add some object-based variation
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # Objects appear closer (subtract depth where dark)
        depth = depth - (binary.astype(float) / 255.0) * 0.1

        return depth.astype(np.float32)

    def predict_grasps(
            self,
            rgb: np.ndarray,
            depth: np.ndarray,
            obj_mask: np.ndarray,
            obj_center: Tuple[int, int]
    ) -> List[GraspPrediction]:
        """
        Predict grasps for an object.

        This is currently a PLACEHOLDER using geometric heuristics.

        To integrate real model (e.g., GraspNet):

        ```python
        # Preprocess
        rgb_tensor = self.preprocess_rgb(rgb)
        depth_tensor = self.preprocess_depth(depth)

        # Run inference
        with torch.no_grad():
            grasp_predictions = self.model.predict(rgb_tensor, depth_tensor)

        # Post-process
        grasps = []
        for pred in grasp_predictions:
            grasp = GraspPrediction()
            grasp.position = pred['position']
            grasp.orientation = pred['rotation']
            grasp.width = pred['width']
            grasp.quality = pred['score']
            grasps.append(grasp)

        return grasps
        ```
        """
        if self.model is not None:
            # Use real model (not implemented yet)
            return self.predict_grasps_with_model(rgb, depth, obj_mask)
        else:
            # Use placeholder (geometric heuristics)
            return self.predict_grasps_placeholder(rgb, depth, obj_mask, obj_center)

    def predict_grasps_placeholder(
            self,
            rgb: np.ndarray,
            depth: np.ndarray,
            obj_mask: np.ndarray,
            obj_center: Tuple[int, int]
    ) -> List[GraspPrediction]:
        """
        Placeholder grasp prediction using geometric heuristics.
        Replace this with real ML model inference.
        """
        grasps = []

        cx, cy = obj_center

        # Get depth at center
        z = depth[cy, cx]

        # Convert to 3D
        x = (cx - self.cx) * z / self.fx
        y = (cy - self.cy) * z / self.fy

        # Generate multiple grasp candidates around object
        num_candidates = 8
        for i in range(num_candidates):
            angle = i * (2 * np.pi / num_candidates)

            # Offset from center
            offset_dist = 0.02  # 2cm offset
            grasp_x = x + offset_dist * np.cos(angle)
            grasp_y = y + offset_dist * np.sin(angle)
            grasp_z = z

            # Create grasp
            grasp = GraspPrediction()
            grasp.position = (grasp_x, grasp_y, grasp_z)

            # Orientation: approach from top, rotate around Z
            grasp.orientation = self.euler_to_quaternion(0, 0, angle)

            # Random width (placeholder)
            grasp.width = 0.05 + np.random.rand() * 0.03  # 5-8cm

            # Quality based on distance from center (placeholder)
            dist_from_center = offset_dist
            grasp.quality = max(0.3, 1.0 - dist_from_center * 5.0)

            # Add noise to quality
            grasp.quality += np.random.randn() * 0.1
            grasp.quality = np.clip(grasp.quality, 0.0, 1.0)

            grasps.append(grasp)

        return grasps

    def euler_to_quaternion(self, roll, pitch, yaw) -> Tuple[float, float, float, float]:
        """Convert Euler angles to quaternion."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (x, y, z, w)

    def publish_grasps(self, grasps: List[GraspPrediction]):
        """Publish all grasp predictions as JSON."""
        grasps_data = []

        for i, grasp in enumerate(grasps):
            grasp_dict = {
                'id': i,
                'position': list(grasp.position),
                'orientation': list(grasp.orientation),
                'width': float(grasp.width),
                'quality': float(grasp.quality),
                'approach_vector': list(grasp.approach_vector),
                'type': grasp.grasp_type,
            }
            grasps_data.append(grasp_dict)

        msg = String()
        msg.data = json.dumps(grasps_data, indent=2)
        self.grasps_pub.publish(msg)

    def publish_best_grasp(self, grasp: GraspPrediction):
        """Publish best grasp as PoseStamped."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"

        msg.pose.position.x = grasp.position[0]
        msg.pose.position.y = grasp.position[1]
        msg.pose.position.z = grasp.position[2]

        msg.pose.orientation.x = grasp.orientation[0]
        msg.pose.orientation.y = grasp.orientation[1]
        msg.pose.orientation.z = grasp.orientation[2]
        msg.pose.orientation.w = grasp.orientation[3]

        self.best_grasp_pub.publish(msg)

        self.get_logger().info(
            f'Best grasp: quality={grasp.quality:.2f}, '
            f'pos=({grasp.position[0]:.3f}, {grasp.position[1]:.3f}, {grasp.position[2]:.3f}), '
            f'width={grasp.width:.3f}m',
            throttle_duration_sec=1.0)

    def visualize_grasps(
            self,
            image: np.ndarray,
            grasps: List[GraspPrediction],
            objects: List
    ) -> np.ndarray:
        """Visualize grasp predictions."""
        vis = image.copy()

        # Draw object contours
        for mask, center in objects:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
            cv2.circle(vis, center, 5, (0, 0, 255), -1)

        # Draw grasps
        for i, grasp in enumerate(grasps):
            # Project 3D position back to 2D
            x, y, z = grasp.position
            u = int(x * self.fx / z + self.cx)
            v = int(y * self.fy / z + self.cy)

            # Color by quality (green = good, red = bad)
            color_r = int(255 * (1 - grasp.quality))
            color_g = int(255 * grasp.quality)
            color = (0, color_g, color_r)

            # Draw grasp point
            cv2.circle(vis, (u, v), 8, color, 2)

            # Draw gripper width indicator
            _, _, _, qw = grasp.orientation
            qz = grasp.orientation[2]
            angle = 2 * np.arctan2(qz, qw)

            width_pixels = int(grasp.width * self.fx / z)
            dx = int(width_pixels / 2 * np.cos(angle))
            dy = int(width_pixels / 2 * np.sin(angle))

            p1 = (u - dx, v - dy)
            p2 = (u + dx, v + dy)
            cv2.line(vis, p1, p2, color, 2)

            # Quality text
            if i == 0:  # Best grasp
                cv2.putText(vis, f'BEST: {grasp.quality:.2f}', (u + 10, v - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                cv2.putText(vis, f'{grasp.quality:.2f}', (u + 10, v - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Header
        header = f'ML Grasp Detector ({self.model_type}) - {len(grasps)} grasps'
        cv2.putText(vis, header, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        warning = 'PLACEHOLDER MODEL - Integrate GraspNet for production'
        cv2.putText(vis, warning, (10, vis.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

        return vis

    def publish_visualization(self, image: np.ndarray):
        """Publish visualization."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.vis_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Vis error: {e}')

    def publish_markers(self, grasps: List[GraspPrediction]):
        """Publish RViz markers."""
        marker_array = MarkerArray()

        for i, grasp in enumerate(grasps):
            # Grasp pose arrow
            marker = Marker()
            marker.header.frame_id = "camera_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasps"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = grasp.position[0]
            marker.pose.position.y = grasp.position[1]
            marker.pose.position.z = grasp.position[2]

            marker.pose.orientation.x = grasp.orientation[0]
            marker.pose.orientation.y = grasp.orientation[1]
            marker.pose.orientation.z = grasp.orientation[2]
            marker.pose.orientation.w = grasp.orientation[3]

            marker.scale.x = 0.1  # Length
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            # Color by quality
            marker.color.r = 1.0 - grasp.quality
            marker.color.g = grasp.quality
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
    node = MLGraspDetector()

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
