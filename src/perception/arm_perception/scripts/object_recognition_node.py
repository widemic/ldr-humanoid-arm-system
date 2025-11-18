#!/usr/bin/env python3
"""
Object Recognition Node for ROS 2

Real-time object detection using YOLOv4 with GPU/CPU support.
Subscribes to camera images, detects objects, and publishes annotated results.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory
from download_yolo_models import YOLOModelDownloader
from config_loader import load_config


class ObjectRecognitionNode(Node):
    """ROS 2 Node for YOLO-based object detection with GPU/CPU support."""

    def __init__(self):
        super().__init__('object_recognition_node')

        # Load configuration from YAML files
        self.base_path = os.path.join(get_package_share_directory('arm_perception'), 'config')
        self.cfg = load_config(self.base_path, logger=self.get_logger())

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            self.cfg.topics.camera_input,
            self.image_callback,
            10
        )

        # Publish processed images
        self.processed_image_pub = self.create_publisher(
            Image,
            self.cfg.topics.processed_output,
            10
        )

        # Download model files if needed
        self.download_models()

        # Setup and load model
        self.setup_device()
        self.net, self.classes, self.detector_type = self.load_model()

        # Cache output layers (calculate once, not every frame)
        if self.net is not None:
            self.output_layers = self._get_output_layers()
        else:
            self.output_layers = []

        # Performance tracking
        self.frame_count = 0
        self.fps = 0.0
        self.last_time = cv2.getTickCount()

        self.get_logger().info(f'Object Recognition started - {self.detector_type}')
        self.get_logger().info(f'Device: {"GPU" if self.gpu_available else "CPU"}')

    def download_models(self):
        """Download YOLO model files if needed."""
        try:
            config_file = os.path.join(self.base_path, 'object_recognition.yaml')
            downloader = YOLOModelDownloader(config_file, logger=self.get_logger())
            downloader.ensure_all_files(include_tiny=True)
        except Exception as e:
            self.get_logger().error(f'Failed to download model files: {e}')
            raise

    def setup_device(self):
        """Configure GPU/CPU computation device."""
        self.gpu_available = False

        if self.cfg.device.use_gpu:
            try:
                cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
                if cuda_devices > 0:
                    cv2.cuda.setDevice(0)
                    device_id = cv2.cuda.getDevice()
                    info = cv2.cuda.DeviceInfo(device_id)

                    self.get_logger().info(f'GPU: {info.name() if hasattr(info, "name") else "Unknown"}')
                    self.get_logger().info(f'Compute: {info.majorVersion()}.{info.minorVersion()}')
                    self.gpu_available = True
                else:
                    self.get_logger().warning('CUDA not available, using CPU')
            except Exception as e:
                self.get_logger().warning(f'GPU setup failed: {e}, using CPU')
        else:
            self.get_logger().info('CPU mode selected')

    def load_model(self):
        """Load YOLO model (YOLOv4 or YOLOv4-tiny fallback)."""
        # Get file paths from config
        weights_path = os.path.join(self.base_path, self.cfg.yolov4.weights)
        config_path = os.path.join(self.base_path, self.cfg.yolov4.config)
        names_path = os.path.join(self.base_path, self.cfg.yolov4.names)

        # Try YOLOv4
        if os.path.exists(weights_path) and os.path.exists(config_path) and os.path.exists(names_path):
            self.get_logger().info(f"Loading YOLOv4...")
            net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

            if self.gpu_available:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                detector_type = "YOLOv4-GPU"
            else:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                detector_type = "YOLOv4-CPU"

            with open(names_path, 'r') as f:
                classes = [line.strip() for line in f.readlines()]

            return net, classes, detector_type

        # Fallback to YOLOv4-tiny
        try:
            weights_path = os.path.join(self.base_path, self.cfg.yolov4_tiny.weights)
            config_path = os.path.join(self.base_path, self.cfg.yolov4_tiny.config)
            names_path = os.path.join(self.base_path, self.cfg.yolov4_tiny.names)

            if os.path.exists(weights_path) and os.path.exists(config_path) and os.path.exists(names_path):
                self.get_logger().info(f"Loading YOLOv4-tiny...")
                net = cv2.dnn.readNetFromDarknet(config_path, weights_path)

                if self.gpu_available:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                    detector_type = "YOLOv4-tiny-GPU"
                else:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU_FP16)
                    detector_type = "YOLOv4-tiny-CPU"

                with open(names_path, 'r') as f:
                    classes = [line.strip() for line in f.readlines()]

                return net, classes, detector_type
        except Exception as e:
            self.get_logger().warning(f"YOLOv4-tiny failed: {e}")

        # No model available
        self.get_logger().error("No YOLO model available!")
        raise RuntimeError("Failed to load any YOLO model")

    def _get_output_layers(self):
        """Get output layer names (called once during initialization)."""
        layer_names = self.net.getLayerNames()
        try:
            unconnected = self.net.getUnconnectedOutLayers()

            if isinstance(unconnected, np.ndarray):
                unconnected = unconnected.flatten().astype(int).tolist()
            elif isinstance(unconnected, (list, tuple)):
                unconnected = [int(i[0]) if isinstance(i, (list, np.ndarray)) else int(i) for i in unconnected]
            else:
                unconnected = [int(unconnected)]

            unconnected = [i for i in unconnected if 1 <= i <= len(layer_names)]

            if not unconnected:
                return layer_names[-3:]
            else:
                return [layer_names[i - 1] for i in unconnected]
        except Exception as e:
            self.get_logger().warning(f"Failed to get output layers: {e}, using last 3 layers")
            return layer_names[-3:]

    def detect_objects(self, image):
        """
        Detect objects in image using YOLO.

        Returns:
            list: Detections with 'box', 'confidence', 'class_name', 'class_id'
        """
        height, width = image.shape[:2]

        try:
            # Preprocess image
            blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.cfg.detection.input_size, self.cfg.detection.input_size),
                                        swapRB=True, crop=False)
            self.net.setInput(blob)

            # Run inference (use cached output layers)
            outputs = self.net.forward(self.output_layers)

            # Process detections
            boxes, confidences, class_ids = [], [], []

            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    if len(scores) == 0:
                        continue

                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    if confidence > self.cfg.detection.confidence_threshold and class_id < len(self.classes):
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        x = max(0, int(center_x - w / 2))
                        y = max(0, int(center_y - h / 2))

                        if w > self.cfg.detection.min_box_size and h > self.cfg.detection.min_box_size:
                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

            # Non-Maximum Suppression
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.cfg.detection.confidence_threshold,
                                      self.cfg.detection.nms_threshold)

            detections = []
            if len(indices) > 0:
                for i in indices.flatten():
                    detections.append({
                        'box': boxes[i],
                        'confidence': confidences[i],
                        'class_name': self.classes[class_ids[i]],
                        'class_id': class_ids[i]
                    })

            return detections

        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
            return []

    def calculate_fps(self):
        """Calculate frames per second."""
        current_time = cv2.getTickCount()
        time_diff = (current_time - self.last_time) / cv2.getTickFrequency()

        if time_diff > self.cfg.performance.fps_update_interval:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_time = current_time

    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            self.frame_count += 1
            self.calculate_fps()

            # Convert ROS message to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Detect and visualize
            processed_image = self.process_image(cv_image)

            # Display (if enabled)
            if self.cfg.visualization.window.show:
                cv2.imshow(self.cfg.visualization.window.title, processed_image)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Quitting...')
                    raise KeyboardInterrupt

            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def process_image(self, image):
        """Detect objects and draw bounding boxes."""
        result = image.copy()

        # Detect objects
        detections = self.detect_objects(image)

        viz = self.cfg.visualization
        draw = viz.drawing

        # Draw detections
        for detection in detections:
            x, y, w, h = detection['box']
            confidence = detection['confidence']
            class_name = detection['class_name']

            color = self.get_color(detection['class_id'])

            # Bounding box
            cv2.rectangle(result, (x, y), (x + w, y + h), color, draw.box_thickness)

            # Label
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, draw.label_font, draw.label_font_scale,
                                        draw.label_font_thickness)[0]
            cv2.rectangle(result, (x, y - label_size[1] - draw.label_padding),
                         (x + label_size[0], y), color, -1)
            cv2.putText(result, label, (x, y - 5), draw.label_font,
                       draw.label_font_scale, draw.label_text_color, draw.label_font_thickness)

            # Center point (if enabled)
            if viz.show_center_point:
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.circle(result, (center_x, center_y), draw.center_point_radius, color, draw.center_point_filled)

                # Coordinates (if enabled)
                if viz.show_coordinates:
                    cv2.putText(result, f"({center_x},{center_y})",
                               (x, y + h + draw.coords_offset_y), draw.label_font,
                               draw.coords_font_scale, color, draw.coords_font_thickness)

        # Info overlay
        if viz.show_fps:
            cv2.putText(result, f'FPS: {self.fps:.1f}', draw.info_fps_position,
                       draw.info_font, draw.info_fps_scale, draw.info_fps_color,
                       draw.info_fps_thickness)

        if viz.show_object_count:
            cv2.putText(result, f'Objects: {len(detections)}', draw.info_count_position,
                       draw.info_font, draw.info_count_scale, draw.info_count_color,
                       draw.info_count_thickness)

        if viz.show_device_info:
            cv2.putText(result, f'{self.detector_type}', draw.info_device_position,
                       draw.info_font, draw.info_device_scale, draw.info_device_color,
                       draw.info_device_thickness)

        if viz.show_quit_message:
            quit_pos = (10, result.shape[0] - draw.info_quit_offset_bottom)
            cv2.putText(result, 'Press Q to quit', quit_pos, draw.info_font,
                       draw.info_quit_scale, draw.info_quit_color, draw.info_quit_thickness)

        return result

    def get_color(self, class_id):
        """Get unique color for each class from config."""
        colors = self.cfg.visualization.colors
        return colors[class_id % len(colors)]


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ObjectRecognitionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
