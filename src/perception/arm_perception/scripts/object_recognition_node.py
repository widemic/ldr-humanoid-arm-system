#!/usr/bin/env python3

"""
GPU/CPU Object Recognition Node for ROS 2
==========================================

This node performs real-time object detection using YOLOv4 models with configurable
GPU or CPU computation. It can automatically download YOLOv4 weights and configuration
files if they are not present.

Features:
- Configurable GPU/CPU computation
- Automatic model downloading
- Real-time object detection with bounding boxes
- Performance monitoring (FPS)
- ROS 2 image topic subscription and publishing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import urllib.request


class GPUObjectRecognitionNode(Node):
    """
    ROS 2 Node for object recognition with configurable GPU/CPU computation.
    
    This node subscribes to a camera image topic, performs object detection
    using YOLOv4 models, and publishes the processed images with bounding boxes.
    The computation device (GPU or CPU) can be configured at initialization.
    """
    
    def __init__(self):
        """Initialize the object recognition node with configurable computation device."""
        super().__init__('object_recognition_node')
        
        # ========== CONFIGURATION SECTION ==========
        # Set USE_GPU = True to use GPU acceleration (if available)
        # Set USE_GPU = False to use CPU only
        self.USE_GPU = False  # Change this value to switch between GPU/CPU
        # ========== END CONFIGURATION ==========
        
        # Initialize CV bridge for ROS 2 - OpenCV image conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Input camera topic
            self.image_callback,        # Callback function for processing
            10                          # Queue size
        )
        
        # Publisher for processed images with detection results
        self.processed_image_pub = self.create_publisher(
            Image, 
            '/camera/color/processed_image',  # Output topic for processed images
            10
        )
        
        # Base path for storing YOLO model files
        self.base_path = '/home/alex/ldr-humanoid-arm-system'
        
        # Configure computation device based on user preference
        self.setup_computation_device()
        
        # Load the appropriate model (YOLOv4 or YOLOv4-tiny) for the selected device
        self.net, self.classes, self.detector_type = self.load_model()
        
        # Performance tracking variables
        self.frame_count = 0           # Count of processed frames
        self.detection_count = 0       # Total objects detected
        self.fps = 0                   # Current frames per second
        self.last_time = cv2.getTickCount()  # Timer for FPS calculation
        
        # Log initialization status
        self.get_logger().info(f'Object Recognition started - Using {self.detector_type}')
        self.get_logger().info(f'Computation device: {"GPU" if self.gpu_available else "CPU"}')
    
    def setup_computation_device(self):
        """
        Configure the computation device based on user preference.
        
        If USE_GPU is True and CUDA is available, configure for GPU computation.
        Otherwise, fall back to CPU computation with appropriate logging.
        """
        self.gpu_available = False  # Initialize as CPU mode
        
        if self.USE_GPU:
            try:
                # Check if CUDA devices are available
                cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
                if cuda_devices > 0:
                    self.get_logger().info(f'Found {cuda_devices} CUDA device(s)')
                    
                    # Set CUDA device 0 as the active device
                    cv2.cuda.setDevice(0)
                    device_id = cv2.cuda.getDevice()
                    
                    # Get device information for logging
                    info = cv2.cuda.DeviceInfo(device_id)
                    
                    # Log GPU device details
                    self.get_logger().info(f'Active GPU device index: {device_id}')
                    self.get_logger().info(f'GPU name: {info.name() if hasattr(info, "name") else "Unknown"}')
                    self.get_logger().info(f'Compute Capability: {info.majorVersion()}.{info.minorVersion()}')
                    self.get_logger().info(f'Total Memory: {getattr(info, "totalGlobalMem", lambda: 0)() // (1024**3)} GB')
                    
                    self.gpu_available = True  # Successfully configured GPU
                else:
                    self.get_logger().warning('CUDA not available, falling back to CPU')
                    self.gpu_available = False

            except Exception as e:
                self.get_logger().warning(f'GPU setup failed: {e}, falling back to CPU')
                self.gpu_available = False
        else:
            self.get_logger().info('CPU mode selected by configuration')
            self.gpu_available = False
    
    def load_model(self):
        """
        Load the object detection model optimized for the selected computation device.
        
        Attempts to load YOLOv4 first, falls back to YOLOv4-tiny if unavailable,
        and provides a basic CPU fallback if both models fail.
        
        Returns:
            tuple: (network, class_names, detector_type)
                - network: OpenCV DNN network object or None
                - class_names: List of class names for detection
                - detector_type: String describing the detector type
        """
        # First attempt: Load YOLOv4 model
        weights_path = os.path.join(self.base_path, 'yolov4.weights')
        config_path = os.path.join(self.base_path, 'yolov4.cfg')
        names_path = os.path.join(self.base_path, 'coco.names')
        
        # Download YOLOv4 if not present
        if not os.path.exists(weights_path):
            self.download_yolov4()
        
        if os.path.exists(weights_path):
            self.get_logger().info(f"Loading YOLOv4 for {'GPU' if self.gpu_available else 'CPU'}...")
            
            # Load the YOLOv4 network from Darknet format
            net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
            
            # Configure network backend based on available device
            if self.gpu_available:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                detector_type = "YOLOv4-GPU"
                self.get_logger().info("YOLOv4 configured for CUDA")
            else:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                detector_type = "YOLOv4-CPU"
                self.get_logger().info("YOLOv4 using CPU")
            
            # Load COCO dataset class names
            with open(names_path, 'r') as f:
                classes = [line.strip() for line in f.readlines()]
            
            return net, classes, detector_type
                
        # Second attempt: Fall back to YOLOv4-tiny if YOLOv4 is unavailable
        try:
            weights_path = os.path.join(self.base_path, 'yolov4-tiny.weights')
            config_path = os.path.join(self.base_path, 'yolov4-tiny.cfg')
            names_path = os.path.join(self.base_path, 'coco.names')
            
            if not os.path.exists(weights_path):
                self.download_yolov4_tiny()
            
            if os.path.exists(weights_path):
                self.get_logger().info(f"Loading YOLOv4-tiny for {'GPU' if self.gpu_available else 'CPU'}...")
                
                net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
                
                # Configure backend for YOLOv4-tiny
                if self.gpu_available:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                    detector_type = "YOLOv4-tiny-GPU"
                else:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                    detector_type = "YOLOv4-tiny-CPU"
                
                # Load COCO class names
                with open(names_path, 'r') as f:
                    classes = [line.strip() for line in f.readlines()]
                
                return net, classes, detector_type
                
        except Exception as e:
            self.get_logger().warning(f"YOLOv4-tiny failed: {e}")
        
        # Final fallback: Basic CPU-based detection
        self.get_logger().info("Using CPU-based fallback detection")
        return None, ["object"], "CPU-Fallback"
    
    def download_yolov4(self):
        """
        Download YOLOv4 model files (weights, config, and class names).
        
        Downloads from official repositories if files are not present locally.
        """
        try:
            self.get_logger().info("Downloading YOLOv4 (245MB)...")
            
            # URLs for YOLOv4 model files
            weights_url = "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights"
            config_url = "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4.cfg"
            names_url = "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names"
            
            # Local file paths
            weights_path = os.path.join(self.base_path, 'yolov4.weights')
            config_path = os.path.join(self.base_path, 'yolov4.cfg')
            names_path = os.path.join(self.base_path, 'coco.names')
            
            # Download model files
            urllib.request.urlretrieve(weights_url, weights_path)
            urllib.request.urlretrieve(config_url, config_path)
            urllib.request.urlretrieve(names_url, names_path)
            
            self.get_logger().info("YOLOv4 downloaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to download YOLOv4: {e}")
    
    def download_yolov4_tiny(self):
        """
        Download YOLOv4-tiny model files.
        
        A smaller, faster version of YOLOv4 suitable for real-time applications.
        """
        try:
            self.get_logger().info("Downloading YOLOv4-tiny...")
            
            # URLs for YOLOv4-tiny model files
            weights_url = "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights"
            config_url = "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg"
            names_url = "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names"
            
            # Local file paths
            weights_path = os.path.join(self.base_path, 'yolov4-tiny.weights')
            config_path = os.path.join(self.base_path, 'yolov4-tiny.cfg')
            names_path = os.path.join(self.base_path, 'coco.names')
            
            # Download model files
            urllib.request.urlretrieve(weights_url, weights_path)
            urllib.request.urlretrieve(config_url, config_path)
            urllib.request.urlretrieve(names_url, names_path)
            
            self.get_logger().info("YOLOv4-tiny downloaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to download YOLOv4-tiny: {e}")
    
    def detect_objects(self, image):
        """
        Perform object detection on the input image using the loaded model.
        
        Args:
            image: OpenCV image (numpy array) to process
            
        Returns:
            list: List of detection dictionaries, each containing:
                - 'box': [x, y, width, height] bounding box coordinates
                - 'confidence': Detection confidence score (0-1)
                - 'class_name': Detected object class name
                - 'class_id': Detected object class ID
        """
        if self.net is None:
            return []  # Return empty list if no model is loaded

        height, width = image.shape[:2]
        detections = []

        try:
            # Preprocess image for the neural network
            # - Normalize pixel values to 0-1 range
            # - Resize to 608x608 (YOLOv4 input size)
            # - Swap Red and Blue channels (OpenCV uses BGR, YOLO expects RGB)
            blob = cv2.dnn.blobFromImage(image, 1/255.0, (608, 608), swapRB=True, crop=False)
            self.net.setInput(blob)

            # Get YOLO layer names
            layer_names = self.net.getLayerNames()

            # Get output layers for YOLO detection
            try:
                unconnected = self.net.getUnconnectedOutLayers()

                # Normalize the output layer indices to a list of integers
                if isinstance(unconnected, np.ndarray):
                    unconnected = unconnected.flatten().astype(int).tolist()
                elif isinstance(unconnected, (list, tuple)):
                    unconnected = [int(i[0]) if isinstance(i, (list, np.ndarray)) else int(i) for i in unconnected]
                else:
                    unconnected = [int(unconnected)]

                # Filter out invalid layer indices
                unconnected = [i for i in unconnected if 1 <= i <= len(layer_names)]

                if not unconnected:
                    self.get_logger().warn("No valid unconnected output layers found — using fallback")
                    output_layers = layer_names[-3:]  # Use last 3 layers as fallback
                else:
                    output_layers = [layer_names[i - 1] for i in unconnected]

            except Exception as e:
                self.get_logger().error(f"Failed to get output layers: {e}")
                output_layers = layer_names[-3:]  # Fallback to generic output layers

            # Perform forward pass through the network (inference)
            outputs = self.net.forward(output_layers)

            # Process detection results
            boxes, confidences, class_ids = [], [], []

            for output in outputs:
                for detection in output:
                    scores = detection[5:]  # Class probabilities start from index 5
                    if len(scores) == 0:
                        continue
                    class_id = np.argmax(scores)  # Get class with highest probability
                    confidence = scores[class_id]  # Get confidence score

                    # Filter detections by confidence threshold and valid class ID
                    if confidence > 0.3 and class_id < len(self.classes):
                        # Convert normalized coordinates to pixel coordinates
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Calculate bounding box coordinates
                        x = max(0, int(center_x - w / 2))
                        y = max(0, int(center_y - h / 2))

                        # Filter out very small detections (likely noise)
                        if w > 10 and h > 10:
                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

            # Apply Non-Maximum Suppression to remove overlapping boxes
            indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.4)
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
        """
        Calculate and update the current frames per second (FPS).
        
        Updates the FPS value every second based on the frame count.
        """
        current_time = cv2.getTickCount()
        time_diff = (current_time - self.last_time) / cv2.getTickFrequency()
        
        if time_diff > 1.0:  # Update FPS every second
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_time = current_time
    
    def image_callback(self, msg):
        """
        Callback function for processing incoming image messages.
        
        Args:
            msg: ROS 2 Image message from the camera topic
        """
        try:
            # Update frame counter and calculate FPS
            self.frame_count += 1
            self.calculate_fps()
            
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the image (object detection and visualization)
            processed_image = self.process_image(cv_image)
            
            # Display the processed image in a window
            cv2.imshow('Object Recognition - Press Q to quit', processed_image)
            
            # Check for 'Q' key press to exit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quitting...')
                raise KeyboardInterrupt
            
            # Convert processed image back to ROS message and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_image(self, image):
        """
        Process the input image by detecting objects and drawing visualization.
        
        Args:
            image: OpenCV image to process
            
        Returns:
            numpy.array: Processed image with bounding boxes and information overlay
        """
        result = image.copy()  # Create a copy to draw on
        
        # Perform object detection
        detections = self.detect_objects(image)
        self.detection_count += len(detections)
        
        # Draw detection results on the image
        for detection in detections:
            x, y, w, h = detection['box']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # Get unique color for each object class
            color = self.get_color_for_class(detection['class_id'])
            
            # Draw bounding box around detected object
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            
            # Create label with class name and confidence
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # Draw background for label text
            cv2.rectangle(result, (x, y - label_size[1] - 10), 
                         (x + label_size[0], y), color, -1)
            
            # Draw label text
            cv2.putText(result, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw center point of the detected object
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(result, (center_x, center_y), 4, color, -1)
            
            # Display coordinates of the center point
            cv2.putText(result, f"({center_x},{center_y})", 
                       (x, y + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Add performance and status information overlay
        cv2.putText(result, f'FPS: {self.fps:.1f}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f'Objects: {len(detections)}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f'Method: {self.detector_type}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result, f'Device: {"GPU" if self.gpu_available else "CPU"}', (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result, 'Press Q to quit', (10, result.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result
    
    def get_color_for_class(self, class_id):
        """
        Generate a unique color for each object class for consistent visualization.
        
        Args:
            class_id: Integer ID of the object class
            
        Returns:
            tuple: BGR color tuple for the class
        """
        # Predefined color palette for different object classes
        colors = [
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (255, 165, 0),  # Orange
            (128, 0, 128),  # Purple
            (255, 192, 203),# Pink
            (0, 128, 128),  # Teal
            (128, 128, 0),  # Olive
            (128, 0, 0),    # Maroon
            (0, 0, 128),    # Navy
            (128, 0, 128),  # Purple
            (0, 128, 0),    # Dark Green
        ]
        return colors[class_id % len(colors)]


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the object recognition node
    node = GPUObjectRecognitionNode()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down object recognition...")
    finally:
        # Cleanup: close windows and destroy node
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()