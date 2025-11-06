#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Try to import mediapipe for better person segmentation; fallback to HOG detector if not present
USE_MEDIAPIPE = False
try:
    import mediapipe as mp
    USE_MEDIAPIPE = True
except Exception:
    USE_MEDIAPIPE = False

class CompleteDepthCamera(Node):
    def __init__(self):
        super().__init__('complete_depth_camera')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile)

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.use_ip_camera = False
        if self.use_ip_camera:
        # Conectare la IP Webcam
            self.phone_ip = "192.168.0.145"  # Înlocuiește cu IP-ul telefonului
            self.phone_port = "8080"
            
            # URL-uri pentru stream-uri
            self.color_stream_url = f"http://{self.phone_ip}:{self.phone_port}/video"
            self.cap = cv2.VideoCapture(self.color_stream_url)
        else:
            self.cap = cv2.VideoCapture(0)

        # Setează rezoluția camerei dacă este suportată
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.bridge = CvBridge()

        # Parametri camerei calibrate pentru rezoluția 640x480
        self.fx = 525.0  # Focal length x
        self.fy = 525.0  # Focal length y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y

        # Variabile pentru tracking și smoothing
        self.previous_depth = None
        self.frame_count = 0

        # Parametri configurabili (tunează după nevoie)
        self.enable_depth_smoothing = True
        self.pointcloud_downsample = 2  # Pas de downsampling pentru pointcloud

        # Depth range
        self.min_depth = 0.30
        self.max_depth = 8.0

        # Person-specific depth (tuneazǎ)
        self.person_near_depth = 0.75  # cât de aproape apare persoana (în metri)
        self.person_center_boost = 0.35  # cât de mult centru e mai aproape (m)

        # Heuristics for contour depth (kept for background objects)
        self.area_to_depth_scale = 180.0

        # Initialize detectors
        if USE_MEDIAPIPE:
            self.get_logger().info('Using MediaPipe SelfieSegmentation for person mask')
            self.mp_selfie = mp.solutions.selfie_segmentation
            # model_selection=1 is for general (0/1 differs small vs large)
            self.segmentation = self.mp_selfie.SelfieSegmentation(model_selection=1)
        else:
            self.get_logger().info('MediaPipe not found — using HOG fallback for person detection')
            # HOG people detector
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.get_logger().info('Complete Depth Camera Node started (person-priority depth)')

    def timer_callback(self):
        ret, color_frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Could not read frame from webcam')
            return

        self.frame_count += 1

        # Resize pentru consistență
        color_frame = cv2.resize(color_frame, (640, 480))

        # Generează depth map bazat pe contururi + persona prioritară
        depth_frame = self.generate_depth_with_person_priority(color_frame)

        # Aplică smoothing temporal dacă este activat
        if self.enable_depth_smoothing:
            depth_frame = self.apply_temporal_smoothing(depth_frame)

        # Publică datele
        self.publish_images(color_frame, depth_frame)
        self.publish_pointcloud(color_frame, depth_frame)
        self.publish_camera_info()

    def generate_depth_with_person_priority(self, color_frame):
        """Estimare depth: detectăm persoana și o forțăm în prim-plan,
           restul scenei rămâne la depth estimat din contururi."""
        height, width = color_frame.shape[:2]

        # Start with far background
        depth_map = np.ones((height, width), dtype=np.float32) * self.max_depth

        # 1) Detect person mask (best effort)
        person_mask = self.detect_person_mask(color_frame)

        if person_mask is not None and np.any(person_mask):
            # Smooth and expand mask a bit to avoid missed edges
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
            person_mask = cv2.morphologyEx(person_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=1)
            person_mask = cv2.dilate(person_mask, kernel, iterations=1)
            person_mask = cv2.GaussianBlur(person_mask.astype(np.float32), (11,11), 0)
            person_mask = (person_mask > 0.2).astype(np.uint8)  # final binary mask

            # Compute centroid of person region
            ys, xs = np.where(person_mask == 1)
            if ys.size > 0:
                cy = int(np.mean(ys))
                cx = int(np.mean(xs))
                # distance from center used to create convex-like shape (center nearer)
                y_coords, x_coords = np.indices((height, width))
                dist = np.sqrt((x_coords - cx)**2 + (y_coords - cy)**2)
                maxd = dist[person_mask==1].max() if np.any(person_mask==1) else 1.0
                maxd = max(maxd, 1.0)
                curvature = (1.0 - (dist / (maxd + 1e-6))) ** 1.8  # sharper falloff
                curvature = np.clip(curvature, 0.0, 1.0)

                # Person depth map: center closer, edges slightly farther
                person_depth_map = self.person_near_depth + (1.0 - curvature) * self.person_center_boost
                person_depth_map = np.clip(person_depth_map, self.min_depth, self.max_depth)

                # Apply to depth_map where person_mask==1 (only if closer)
                replace_idx = (person_mask == 1) & (person_depth_map < depth_map)
                depth_map[replace_idx] = person_depth_map[replace_idx]
        # else: no person detected — fallback to contour-only

        # 2) Contour-based depth for other objects (background furniture etc.)
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        contours = self.detect_objects(gray)

        # Sort contours by area descending (big objects first)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 400:
                continue
            # Make mask for contour
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, color=255, thickness=-1)
            if np.sum(mask) == 0:
                continue
            # centroid
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            inv_area = 1.0 / (np.sqrt(area) + 1e-6)
            depth_from_area = self.area_to_depth_scale * inv_area
            depth_from_pos = (1.0 - (cy / float(height))) * 1.2
            base_depth = depth_from_area + (1.0 - depth_from_pos)
            base_depth = float(np.clip(base_depth, self.min_depth, self.max_depth))

            # Convex-like shape inside contour (gives impression of volume)
            y_coords, x_coords = np.indices((height, width))
            dist_from_centroid = np.sqrt((x_coords - cx)**2 + (y_coords - cy)**2)
            max_dist = np.max(dist_from_centroid[mask == 255]) if np.any(mask == 255) else 1.0
            if max_dist <= 0:
                max_dist = 1.0
            curvature = (1.0 - (dist_from_centroid / (max_dist + 1e-6))) ** 1.5
            curvature = np.clip(curvature, 0.0, 1.0)
            obj_depth = base_depth - (curvature * 0.25)
            obj_depth = np.clip(obj_depth, self.min_depth, self.max_depth)

            replace_mask = (mask == 255) & (obj_depth < depth_map)
            depth_map[replace_mask] = obj_depth[replace_mask]

        # Gentle spatial smoothing and small realistic noise
        depth_map = cv2.GaussianBlur(depth_map, (5,5), 0)
        noise = np.random.normal(0, 0.008, depth_map.shape)
        depth_map = np.clip(depth_map + depth_map * noise, self.min_depth, self.max_depth)

        return depth_map.astype(np.float32)

    def detect_person_mask(self, frame_bgr):
        """Return binary mask (uint8) where person is 1. Uses MediaPipe if available,
           otherwise HOG fallback returns bounding-box mask(s)."""
        h, w = frame_bgr.shape[:2]
        if USE_MEDIAPIPE:
            # Mediapipe expects RGB
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            results = self.segmentation.process(frame_rgb)
            if results and hasattr(results, 'segmentation_mask') and results.segmentation_mask is not None:
                mask = results.segmentation_mask  # float mask [0,1]
                # Resize if needed
                if mask.shape != (h, w):
                    mask = cv2.resize(mask, (w, h))
                # threshold to binary
                bin_mask = (mask > 0.5).astype(np.uint8)
                return bin_mask
            return None
        else:
            # HOG fallback: returns mask from detected bounding boxes
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            # detectMultiScale is slower; use default params tuned for people detection
            rects, weights = self.hog.detectMultiScale(gray, winStride=(8,8), padding=(8,8), scale=1.05)
            if len(rects) == 0:
                return None
            mask = np.zeros((h, w), dtype=np.uint8)
            for (x, y, rw, rh) in rects:
                # Expand box slightly to include head/shoulders
                pad_x = int(rw * 0.15)
                pad_y = int(rh * 0.08)
                x1 = max(0, x - pad_x)
                y1 = max(0, y - pad_y)
                x2 = min(w, x + rw + pad_x)
                y2 = min(h, y + rh + pad_y)
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            return (mask > 0).astype(np.uint8)

    def detect_objects(self, gray_frame):
        """Detectează obiecte folosind threshold și operații morfologice."""
        blurred = cv2.GaussianBlur(gray_frame, (7, 7), 1.5)
        binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 11, 2)
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        # Remove small blobs
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
        cleaned = np.zeros_like(binary)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 300:
                cleaned[labels == i] = 255
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def apply_temporal_smoothing(self, current_depth):
        """Aplică smoothing temporal pentru depth frames (IIR)."""
        if self.previous_depth is None:
            self.previous_depth = current_depth.copy()
            return current_depth
        alpha = 0.35
        smoothed_depth = alpha * current_depth + (1.0 - alpha) * self.previous_depth
        self.previous_depth = smoothed_depth.copy()
        return smoothed_depth

    def publish_images(self, color_frame, depth_frame):
        """Publică imaginile RGB și depth"""
        timestamp = self.get_clock().now().to_msg()

        # Publică imaginea color
        color_msg = self.bridge.cv2_to_imgmsg(color_frame, 'bgr8')
        color_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'camera_link'
        self.rgb_pub.publish(color_msg)

        # Publică imaginea de depth: folosim 32-bit float single channel
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame.astype(np.float32), encoding='32FC1')
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = 'camera_link'
        self.depth_pub.publish(depth_msg)

    def publish_pointcloud(self, color_frame, depth_frame):
        """Publică pointcloud-ul bazat pe depth_frame estimat."""
        height, width = depth_frame.shape

        step = max(1, int(self.pointcloud_downsample))
        u_coords = np.arange(0, width, step, dtype=np.int32)
        v_coords = np.arange(0, height, step, dtype=np.int32)
        u_grid, v_grid = np.meshgrid(u_coords, v_coords)

        depth_sampled = depth_frame[v_grid, u_grid]
        color_sampled = color_frame[v_grid, u_grid]

        valid_mask = (depth_sampled > self.min_depth) & (depth_sampled < self.max_depth)
        if not np.any(valid_mask):
            return

        u_valid = u_grid[valid_mask].astype(np.float32)
        v_valid = v_grid[valid_mask].astype(np.float32)
        z_valid = depth_sampled[valid_mask].astype(np.float32)

        x = (u_valid - self.cx) * z_valid / self.fx
        y = (v_valid - self.cy) * z_valid / self.fy

        color_valid = color_sampled[valid_mask]
        r = (color_valid[:, 2]).astype(np.uint8)
        g = (color_valid[:, 1]).astype(np.uint8)
        b = (color_valid[:, 0]).astype(np.uint8)

        points_array = np.column_stack([x, y, z_valid])

        if len(points_array) > 50:
            distances = np.linalg.norm(points_array, axis=1)
            median_dist = np.median(distances)
            mad = np.median(np.abs(distances - median_dist)) + 1e-6
            keep = np.abs(distances - median_dist) < 3.0 * mad
            points_array = points_array[keep]
            r = r[keep]; g = g[keep]; b = b[keep]

        if len(points_array) == 0:
            return

        cloud_data = []
        for i in range(len(points_array)):
            px, py, pz = points_array[i]
            rgb_uint = struct.unpack('I', struct.pack('BBBB', int(b[i]), int(g[i]), int(r[i]), 0))[0]
            rgb_as_float = struct.unpack('f', struct.pack('I', rgb_uint))[0]
            cloud_data.append([float(px), float(py), float(pz), float(rgb_as_float)])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pointcloud = point_cloud2.create_cloud(header, fields, cloud_data)
        self.pointcloud_pub.publish(pointcloud)

        if self.frame_count % 30 == 0:
            self.get_logger().info(f'PointCloud: {len(points_array)} points (sampled)')

    def publish_camera_info(self):
        """Publică informațiile camerei"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_link'
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
        """Cleanup la shutdown"""
        self.get_logger().info('Shutting down depth camera node')
        if self.cap.isOpened():
            self.cap.release()
        # If mediapipe was used, close it gracefully
        try:
            if USE_MEDIAPIPE and hasattr(self, 'segmentation') and self.segmentation is not None:
                self.segmentation.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = CompleteDepthCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
