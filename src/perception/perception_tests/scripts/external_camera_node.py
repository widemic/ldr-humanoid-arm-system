#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ExternalCameraNode(Node):
    """Simple node that republishes raw frames from an external camera."""

    def __init__(self):
        super().__init__('external_camera_node')

        self.declare_parameter('camera_namespace', '/external/camera')
        self.camera_namespace = self.get_parameter('camera_namespace').value or '/external/camera'
        if not self.camera_namespace.startswith('/'):
            self.camera_namespace = '/' + self.camera_namespace
        self.camera_namespace = self.camera_namespace.rstrip('/')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        color_topic = f'{self.camera_namespace}/color/image_raw'
        info_topic = f'{self.camera_namespace}/color/camera_info'

        self.rgb_pub = self.create_publisher(Image, color_topic, qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, info_topic, qos_profile)

        self.bridge = CvBridge()

        # Configure camera source (change index/IP stream if needed).
        self.use_ip_camera = False
        if self.use_ip_camera:
            phone_ip = "192.168.0.145"
            phone_port = "8080"
            color_stream_url = f"http://{phone_ip}:{phone_port}/video"
            self.cap = cv2.VideoCapture(color_stream_url)
        else:
            self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Static intrinsics that match the configured resolution.
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.frame_id = f"{self.camera_namespace.strip('/').replace('/', '_')}_link"
        self.get_logger().info(
            f'External camera node started (RGB stream only) publishing on {color_topic}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Could not read frame from camera')
            return

        frame = cv2.resize(frame, (640, 480))
        timestamp = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        color_msg.header.stamp = timestamp
        color_msg.header.frame_id = self.frame_id
        self.rgb_pub.publish(color_msg)

        info_msg = CameraInfo()
        info_msg.header.stamp = timestamp
        info_msg.header.frame_id = self.frame_id
        info_msg.height = 480
        info_msg.width = 640
        info_msg.distortion_model = 'plumb_bob'
        info_msg.k = [self.fx, 0.0, self.cx,
                      0.0, self.fy, self.cy,
                      0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        info_msg.p = [self.fx, 0.0, self.cx, 0.0,
                      0.0, self.fy, self.cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down external camera node')
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ExternalCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
