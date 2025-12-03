#!/usr/bin/env python3
"""
YOLOv8 Native Object Tracking
Based on: https://github.com/computervisioneng/object-tracking-yolov8-native

This script uses YOLOv8's built-in tracking capabilities to detect and track objects
in video streams (camera, video file, or ROS 2 topic).

ROS 2 Integration:
- Set USE_ROS2=true to enable ROS 2 topic subscription
- Set ROS2_TOPIC=/camera/color/image_raw (default) or custom topic
"""

from ultralytics import YOLO
import cv2
import sys
import os
import tempfile
from pathlib import Path
import math
import numpy as np

import yaml

# ROS 2 support (enabled by default)
USE_ROS2 = os.environ.get("USE_ROS2", "true").lower() in {"1", "true", "t", "yes", "y", "on"}
if USE_ROS2:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        ROS2_AVAILABLE = True
    except ImportError:
        print("Warning: ROS 2 libraries not available. Install with:")
        print("  pip install rclpy cv-bridge")
        print("Falling back to camera/video mode.")
        USE_ROS2 = False
        ROS2_AVAILABLE = False
else:
    ROS2_AVAILABLE = False


def _str_to_bool(val: str) -> bool:
    return str(val).lower() in {"1", "true", "t", "yes", "y", "on"}


# ROS 2 Image Subscriber (thread-safe frame provider)
class ROS2CameraNode(Node):
    """ROS 2 node that subscribes to camera topic and provides frames."""

    def __init__(self, topic="/camera/color/image_raw"):
        super().__init__('yolo_tracking_camera_subscriber')
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_lock = __import__('threading').Lock()

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {topic}')

    def image_callback(self, msg):
        """Convert ROS Image message to OpenCV format."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.current_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def get_frame(self):
        """Thread-safe frame retrieval."""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None


# Object validation based on physical characteristics
class ObjectValidator:
    """Validate object classifications based on physical characteristics."""

    # Expected size ranges (in pixels) for common objects at typical viewing distances
    # Format: class_name -> (min_area, max_area, min_aspect_ratio, max_aspect_ratio)
    # Aspect ratio = width / height
    OBJECT_CONSTRAINTS = {
        # Electronics - portable devices
        "cell phone": (1000, 100000, 0.08, 15.0),   # Phone: very permissive (edge cases, occlusion)
        "mouse": (800, 25000, 0.3, 5.0),            # Mouse: small, rectangular
        "remote": (1000, 40000, 0.08, 15.0),        # Remote: very permissive like phone
        "keyboard": (15000, 100000, 1.5, 4.0),      # Keyboard: medium, wide rectangle

        # Electronics - larger devices
        "laptop": (15000, 250000, 0.8, 2.5),        # Laptop: medium-large, landscape (relaxed for distance)
        "tv": (50000, 400000, 1.2, 2.5),            # TV: very large, landscape

        # Kitchen items - small
        "bottle": (3000, 40000, 0.25, 0.6),         # Bottle: small, tall portrait
        "cup": (2000, 30000, 0.6, 1.4),             # Cup: small, roughly square
        "wine glass": (2000, 25000, 0.3, 0.7),      # Glass: small, tall portrait
        "fork": (500, 8000, 0.1, 0.4),              # Fork: tiny, very elongated
        "knife": (500, 10000, 0.1, 0.4),            # Knife: tiny, very elongated
        "spoon": (500, 8000, 0.15, 0.5),            # Spoon: tiny, elongated
        "bowl": (5000, 50000, 0.8, 1.3),            # Bowl: medium, circular

        # Furniture - large objects
        "chair": (40000, 300000, 0.4, 1.2),         # Chair: large, portrait/square
        "couch": (80000, 500000, 1.5, 3.5),         # Couch: very large, wide
        "dining table": (60000, 400000, 1.0, 3.0),  # Table: very large, varies
        "bed": (100000, 600000, 1.2, 2.5),          # Bed: huge, landscape

        # Accessories
        "backpack": (10000, 80000, 0.6, 1.2),       # Backpack: medium, portrait/square
        "handbag": (5000, 50000, 0.6, 1.5),         # Handbag: small-medium, varies
        "suitcase": (20000, 150000, 0.6, 1.4),      # Suitcase: large, portrait/square

        # Small items
        "toothbrush": (300, 3000, 0.15, 0.35),      # Toothbrush: TINY, very elongated (stricter to avoid phone misclassification)
        "scissors": (1000, 15000, 0.3, 1.0),        # Scissors: small, varies
        "book": (5000, 50000, 0.5, 1.2),            # Book: medium, portrait/landscape
    }

    @staticmethod
    def is_valid_classification(class_name, width, height):
        """Check if object size and aspect ratio match expected class characteristics.

        Returns:
            tuple: (is_valid: bool, reason: str)
        """
        if class_name not in ObjectValidator.OBJECT_CONSTRAINTS:
            # Unknown class - accept it (no constraints defined)
            return True, "no constraints"

        area = width * height
        aspect_ratio = width / height if height > 0 else 1.0

        min_area, max_area, min_aspect, max_aspect = ObjectValidator.OBJECT_CONSTRAINTS[class_name]

        # Check area constraint
        area_valid = min_area <= area <= max_area
        # Check aspect ratio constraint
        aspect_valid = min_aspect <= aspect_ratio <= max_aspect

        if area_valid and aspect_valid:
            return True, "valid"

        # Build rejection reason
        reasons = []
        if not area_valid:
            reasons.append(f"area {int(area)}px² not in [{int(min_area)}, {int(max_area)}]")
        if not aspect_valid:
            reasons.append(f"aspect {aspect_ratio:.2f} not in [{min_aspect:.2f}, {max_aspect:.2f}]")

        return False, "; ".join(reasons)

    @staticmethod
    def get_plausible_classes(width, height):
        """Get list of plausible object classes based on size and aspect ratio.

        Returns:
            list: List of (class_name, score) tuples, sorted by priority
        """
        area = width * height
        aspect_ratio = width / height if height > 0 else 1.0

        # Priority order: electronics > kitchen > accessories > furniture
        priority_order = {
            # Electronics (highest priority - most commonly tracked)
            "cell phone": 100, "laptop": 95, "mouse": 90, "keyboard": 88, "remote": 85, "tv": 80,
            # Kitchen items
            "bottle": 70, "cup": 68, "wine glass": 65, "bowl": 60, "fork": 55, "knife": 55, "spoon": 55,
            # Accessories
            "backpack": 50, "handbag": 48, "suitcase": 45, "book": 43,
            # Small items
            "scissors": 40, "toothbrush": 35,
            # Furniture (lowest priority - unlikely to track)
            "chair": 20, "couch": 18, "dining table": 15, "bed": 10,
        }

        plausible = []
        for class_name, (min_area, max_area, min_aspect, max_aspect) in ObjectValidator.OBJECT_CONSTRAINTS.items():
            # Check if object matches constraints
            area_match = min_area <= area <= max_area
            aspect_match = min_aspect <= aspect_ratio <= max_aspect

            # Both must match for object to be plausible
            if area_match and aspect_match:
                priority = priority_order.get(class_name, 50)  # Default medium priority
                plausible.append((class_name, priority))

        # Sort by priority (higher first)
        plausible.sort(key=lambda x: x[1], reverse=True)

        return plausible


# Orientation and cropping classes (original implementation)
class OrientationCalculator:
    """Calculate object orientation using PCA (inspired by track_object.py)."""

    def __init__(self, smoothing=0.15, max_jump=50):
        self.smoothing = smoothing
        self.max_jump = max_jump
        self.last_angle = None

    def calculate(self, mask, frame_shape):
        """Calculate orientation angle from segmentation mask using PCA."""
        try:
            # Resize mask to match frame dimensions
            mask_resized = cv2.resize(mask, (frame_shape[1], frame_shape[0]))
            mask_uint8 = (mask_resized * 255).astype(np.uint8)

            # Find contours
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, None, None

            # Get largest contour
            contour = max(contours, key=cv2.contourArea)
            if len(contour) < 5:
                return None, None, None

            # Perform PCA to find principal axis
            pts = contour.reshape(-1, 2).astype(np.float32)
            mean, eigenvectors, _ = cv2.PCACompute2(pts, mean=None)
            cx, cy = mean[0]

            # Get angle of major axis
            vx, vy = eigenvectors[0]
            angle_deg = math.degrees(math.atan2(vy, vx))

            # Convert to rotation needed to make vertical
            rotation = 90.0 - angle_deg
            while rotation > 90:
                rotation -= 180
            while rotation < -90:
                rotation += 180

            # Smooth the angle
            if self.last_angle is not None:
                delta = rotation - self.last_angle
                if abs(delta) > self.max_jump:
                    rotation = self.last_angle + math.copysign(self.max_jump, delta)
                rotation = (1 - self.smoothing) * self.last_angle + self.smoothing * rotation

            self.last_angle = rotation
            return rotation, float(cx), float(cy)

        except Exception as e:
            print(f"Error calculating orientation: {e}")
            return None, None, None

    def reset(self):
        """Reset smoothing state."""
        self.last_angle = None


class UprightCropper:
    """Create upright-rotated crop of objects."""

    def __init__(self, padding=60):
        self.padding = padding

    def create_crop(self, frame, bbox, angle, label, track_id):
        """Create upright crop with rotation and overlays."""
        x1, y1, x2, y2 = map(int, bbox)

        px1 = max(0, x1 - self.padding)
        py1 = max(0, y1 - self.padding)
        px2 = min(frame.shape[1], x2 + self.padding)
        py2 = min(frame.shape[0], y2 + self.padding)

        region = frame[py1:py2, px1:px2].copy()
        if region.size == 0:
            return None

        if angle is not None:
            region = self._rotate_image(region, -angle)

        region = self._add_info_overlay(region, label, track_id, angle)
        region = self._add_alignment_guides(region)

        max_h = 600
        h, w = region.shape[:2]
        if h > max_h:
            scale = max_h / h
            region = cv2.resize(region, (int(w * scale), max_h))

        return region

    def _rotate_image(self, img, angle):
        """Rotate image around center."""
        h, w = img.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        cos = abs(M[0, 0])
        sin = abs(M[0, 1])
        new_w = int(h * sin + w * cos)
        new_h = int(h * cos + w * sin)
        M[0, 2] += (new_w - w) / 2
        M[1, 2] += (new_h - h) / 2
        return cv2.warpAffine(img, M, (new_w, new_h),
                             borderMode=cv2.BORDER_CONSTANT,
                             borderValue=(40, 40, 40))

    def _add_info_overlay(self, img, label, track_id, angle):
        """Add semi-transparent info bar."""
        h, w = img.shape[:2]
        overlay = img.copy()
        bar_h = 100
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.6, img, 0.4, 0)

        y_pos = 30
        cv2.putText(img, f"{label.upper()}", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (50, 255, 50), 3)
        y_pos += 35
        cv2.putText(img, f"Track ID: {track_id}", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 200, 255), 2)
        if angle is not None:
            y_pos += 30
            cv2.putText(img, f"Angle: {angle:.1f}°", (20, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 100), 2)
        return img

    def _add_alignment_guides(self, img):
        """Add center crosshairs and up arrow."""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx, 0), (cx, h), (0, 255, 255), 3)
        cv2.line(img, (0, cy), (w, cy), (80, 80, 80), 1)
        arrow_top = max(150, cy - 100)
        arrow_bot = arrow_top + 50
        if arrow_top > 120:
            cv2.arrowedLine(img, (cx, arrow_bot), (cx, arrow_top),
                           (0, 255, 255), 4, tipLength=0.4)
            cv2.putText(img, "UP", (cx - 30, arrow_top - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return img


def prepare_tracker_config(tracker_path: str, fps: int) -> str:
    """
    Load and optionally override ByteTrack tracker settings to improve ID
    persistence for fast-moving objects. If the tracker file cannot be read,
    falls back to the original path.
    """

    if not os.path.exists(tracker_path):
        return tracker_path

    try:
        with open(tracker_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except Exception as exc:
        print(f"Warning: could not load tracker config '{tracker_path}': {exc}")
        return tracker_path

    tracker_type_lower = str(cfg.get("tracker_type", "")).lower()
    if tracker_type_lower not in ["bytetrack", "botsort"]:
        return tracker_path

    def env_override(name, cast, default):
        val = os.environ.get(name)
        if val is None:
            return default
        try:
            return cast(val)
        except ValueError:
            print(f"Warning: invalid {name}={val}, using default {default}")
            return default

    # Allow runtime overrides
    cfg["track_high_thresh"] = env_override(
        "TRACK_HIGH_THRESH", float, cfg.get("track_high_thresh")
    )
    cfg["track_low_thresh"] = env_override(
        "TRACK_LOW_THRESH", float, cfg.get("track_low_thresh")
    )
    cfg["new_track_thresh"] = env_override(
        "NEW_TRACK_THRESH", float, cfg.get("new_track_thresh")
    )
    cfg["fuse_score"] = env_override(
        "FUSE_SCORE",
        lambda v: _str_to_bool(v),
        cfg.get("fuse_score", True),
    )

    track_buffer_default = cfg.get("track_buffer", 30)
    persist_seconds = env_override("TRACK_PERSIST_SECS", float, None)
    if fps and fps > 0:
        if persist_seconds is not None:
            track_buffer_default = max(track_buffer_default, int(fps * persist_seconds))
        else:
            track_buffer_default = max(track_buffer_default, int(fps * 8))  # keep IDs ~8s (LONGER!)
    cfg["track_buffer"] = env_override("TRACK_BUFFER", int, track_buffer_default)
    cfg["match_thresh"] = env_override("MATCH_THRESH", float, cfg.get("match_thresh", 0.3))  # LOWER threshold - easier matching

    if fps and fps > 0:
        cfg["frame_rate"] = fps

    # Write modified config to a temp file to avoid clobbering the original
    tracker_name = "botsort" if tracker_type_lower == "botsort" else "bytetrack"
    tmp_tracker = Path(tempfile.gettempdir()) / f"{tracker_name}_runtime.yaml"
    try:
        with open(tmp_tracker, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f)
        print(
            f"Using tuned tracker config: {tmp_tracker} "
            f"(track_buffer={cfg.get('track_buffer')}, match_thresh={cfg.get('match_thresh')})"
        )
        return str(tmp_tracker)
    except Exception as exc:
        print(f"Warning: could not write tuned tracker config: {exc}")
        return tracker_path


def resolve_model_name() -> str:
    """Pick YOLO model: env override, prefer YOLOv11 locally, fallback to YOLOv8 defaults."""
    env_model = os.environ.get("YOLO_MODEL")
    if env_model:
        return env_model

    candidates = [
        # YOLOv11 (preferred if present)
        "yolov11n-seg.pt",
        "yolov11s-seg.pt",
        "yolov11n.pt",
        "models/yolov11n-seg.pt",
        "models/yolov11s-seg.pt",
        "models/yolov11n.pt",
        # YOLOv8 fallbacks
        "yolov8x-seg.pt",
        "yolov8n-seg.pt",
        "models/yolov8x-seg.pt",
        "models/yolov8n-seg.pt",
    ]

    for candidate in candidates:
        if Path(candidate).exists():
            return candidate

    # Final fallback if nothing is present
    return "yolov8x-seg.pt"


def _bbox_iou(box_a, box_b):
    """Compute IoU for two [x1, y1, x2, y2] boxes."""
    xa1, ya1, xa2, ya2 = box_a
    xb1, yb1, xb2, yb2 = box_b
    inter_x1, inter_y1 = max(xa1, xb1), max(ya1, yb1)
    inter_x2, inter_y2 = min(xa2, xb2), min(ya2, yb2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = max(0.0, xa2 - xa1) * max(0.0, ya2 - ya1)
    area_b = max(0.0, xb2 - xb1) * max(0.0, yb2 - yb1)
    union = area_a + area_b - inter_area
    return inter_area / union if union > 0 else 0.0


def _dedupe_overlapping_boxes(boxes, locked_track_id, iou_thr=0.6):
    """Remove overlapping detections to avoid duplicate tracks on same object.

    Keeps highest-conf boxes; always keeps the locked track if present.
    Returns a set of indices to keep.
    """
    if boxes is None or len(boxes) == 0 or boxes.xyxy is None:
        return set()

    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy() if boxes.conf is not None else np.zeros(len(xyxy))
    ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else np.arange(len(xyxy))

    order = np.argsort(-conf)  # high to low
    keep = []

    for idx in order:
        if locked_track_id is not None and ids[idx] == locked_track_id:
            keep.append(idx)
            continue
        box = xyxy[idx]
        overlaps = False
        for kept_idx in keep:
            if _bbox_iou(box, xyxy[kept_idx]) > iou_thr:
                overlaps = True
                break
        if not overlaps:
            keep.append(idx)

    return set(keep)


def _mask_iou(mask_a, mask_b):
    """Compute IoU between two binary masks (numpy arrays of 0/1 or bool)."""
    if mask_a is None or mask_b is None:
        return 0.0
    inter = np.logical_and(mask_a, mask_b).sum()
    union = np.logical_or(mask_a, mask_b).sum()
    return float(inter) / float(union) if union > 0 else 0.0


def _create_cv_tracker():
    """Try to create a robust OpenCV tracker (CSRT/KCF/MOSSE)."""
    candidates = [
        ("CSRT", lambda: cv2.TrackerCSRT_create()),
        ("KCF", lambda: cv2.TrackerKCF_create()),
        ("legacy CSRT", lambda: cv2.legacy.TrackerCSRT_create()),
        ("legacy KCF", lambda: cv2.legacy.TrackerKCF_create()),
        ("legacy MOSSE", lambda: cv2.legacy.TrackerMOSSE_create()),
    ]
    for name, fn in candidates:
        try:
            tracker = fn()
            print(f"Using {name} tracker (OpenCV) for fallback.")
            return tracker
        except Exception:
            continue
    print("Warning: No OpenCV tracker available for fallback.")
    return None


def main():
    """Main tracking application using YOLOv8 native tracking."""

    # Configuration
    # Prefer YOLOv11 if present locally, otherwise use YOLOv8x-seg for maximum accuracy
    model_name = resolve_model_name()
    video_source = os.environ.get("VIDEO_SOURCE", "0")  # 0 for webcam, or path to video file
    ros2_topic = os.environ.get("ROS2_TOPIC", "/camera/color/image_raw")  # ROS 2 topic

    # Tracking parameters - ULTRA-AGGRESSIVE FOR FAST MOTION
    conf_threshold = float(os.environ.get("CONF_THRESHOLD", "0.20"))  # Slightly higher to reduce false positives
    iou_threshold = float(os.environ.get("IOU_THRESHOLD", "0.05"))    # VERY low IoU - accepts large motion
    max_age = int(os.environ.get("MAX_AGE", "200"))                   # Keep tracks LONGER - 6+ seconds
    min_hits = int(os.environ.get("MIN_HITS", "1"))                   # Instant confirmation

    # Use BoT-SORT with ReID for superior tracking
    tracker_type = os.environ.get("TRACKER", "botsort_aggressive.yaml")  # BoT-SORT with ReID enabled
    use_half_precision = _str_to_bool(os.environ.get("USE_HALF", "false"))  # FP16 for speed

    # Higher FPS for smoother tracking
    force_fps = int(os.environ.get("FORCE_FPS", "0"))                 # 0 = auto, 30/60 = force

    print("=" * 70)
    print("  YOLOv8 + BoT-SORT - ULTIMATE PERSISTENT TRACKING")
    print("=" * 70)
    print(f"\nModel: {model_name}")
    if USE_ROS2:
        print(f"Input: ROS 2 Topic '{ros2_topic}'")
    else:
        print(f"Video source: {video_source}")
    print(f"Confidence threshold: {conf_threshold}")
    print(f"IoU threshold: {iou_threshold} (VERY LOW - aggressive matching)")
    print(f"Max age: {max_age} frames (8+ seconds persistence)")
    print(f"Min hits: {min_hits} (instant confirmation)")
    print(f"Tracker: {tracker_type}")
    print(f"Half precision (FP16): {use_half_precision}")
    print("\nFeatures:")
    print("  ✓ BoT-SORT with ReID - appearance-based re-identification")
    print("  ✓ Global Motion Compensation (sparseOptFlow)")
    print("  ✓ SHORT motion trails (30 points) with THICK lines")
    print("  ✓ Velocity-based object selection")
    print("  ✓ Auto-locks onto MOST MOVING object")
    print("  ✓ ULTRA-PERSISTENT tracking (maintains ID through occlusions)")
    print("  ✓ Physical validation - rejects implausible classifications")
    print("  ✓ Upright window: Rotated crop keeping object vertical")
    print("  ✓ Orientation arrow: Shows object tilt angle")
    if USE_ROS2:
        print("  ✓ ROS 2 Integration - subscribes to camera topics")
    print("\nControls:")
    print("  - Press 'q' to quit")
    print("  - Press 'p' to pause/resume")
    print("  - Press '+' to increase confidence")
    print("  - Press '-' to decrease confidence")
    print("  - Press 'l' to unlock/release current object")
    print("  - Press 'r' to reset tracking state and clear trails")
    print("\nNote: System auto-locks onto MOST MOVING object after 5 frames")
    print("=" * 70 + "\n")

    # Load YOLOv8 model
    print("Loading YOLOv8 model...")
    try:
        model = YOLO(model_name)
        print("✓ Model loaded successfully!\n")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        print("\nMake sure you have ultralytics installed:")
        print("  pip install ultralytics")
        sys.exit(1)

    # Initialize video source or ROS 2 node
    ros2_node = None
    ros2_executor = None
    cap = None

    if USE_ROS2:
        # Initialize ROS 2
        print(f"Initializing ROS 2 node and subscribing to {ros2_topic}...")
        rclpy.init()
        ros2_node = ROS2CameraNode(topic=ros2_topic)

        # Spin ROS 2 in background thread
        import threading
        ros2_executor = rclpy.executors.SingleThreadedExecutor()
        ros2_executor.add_node(ros2_node)
        ros2_thread = threading.Thread(target=ros2_executor.spin, daemon=True)
        ros2_thread.start()
        print("✓ ROS 2 node initialized and spinning!\n")

        # Wait for first frame
        print("Waiting for first frame from ROS 2 topic...")
        import time
        timeout = 10  # seconds
        start_time = time.time()
        while ros2_node.get_frame() is None:
            if time.time() - start_time > timeout:
                print(f"✗ No frames received from {ros2_topic} after {timeout}s")
                print("\nTroubleshooting:")
                print(f"  - Check if topic exists: ros2 topic list | grep {ros2_topic}")
                print(f"  - Check topic type: ros2 topic info {ros2_topic}")
                print(f"  - Echo topic: ros2 topic echo {ros2_topic} --max-count 1")
                rclpy.shutdown()
                sys.exit(1)
            time.sleep(0.1)
        print("✓ Receiving frames from ROS 2!\n")

        # Get frame properties from first frame
        first_frame = ros2_node.get_frame()
        height, width = first_frame.shape[:2]
        fps = 30  # Assume 30 FPS for ROS 2 topics
    else:
        # Open video source
        # If video_source is a digit string, convert to int for camera
        if video_source.isdigit():
            video_source = int(video_source)

        print(f"Opening video source: {video_source}...")
        cap = cv2.VideoCapture(video_source)

        # Force 1080p on camera sources to stabilize detection/tracking
        desired_width, desired_height = 1920, 1080

        # Optionally set camera pixel format (helps unlock higher resolutions)
        if isinstance(video_source, int):
            fourcc_str = os.environ.get("CAM_FOURCC", "MJPG")  # Common: MJPG, YUYV
            if fourcc_str:
                try:
                    fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
                    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
                    print(f"Requesting camera FOURCC={fourcc_str} for higher resolution")
                except Exception as exc:
                    print(f"Warning: could not set FOURCC {fourcc_str}: {exc}")
        if isinstance(video_source, int):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        # Set higher FPS if specified
        if force_fps > 0 and isinstance(video_source, int):
            cap.set(cv2.CAP_PROP_FPS, force_fps)
            print(f"Requesting {force_fps} FPS from camera...")

        if not cap.isOpened():
            print(f"✗ Unable to open video source: {video_source}")
            print("\nTroubleshooting:")
            print("  - For webcam: try VIDEO_SOURCE=0, 1, or 2")
            print("  - For video file: provide full path to the file")
            print("  - Camera permission issue? Check if you're in video group:")
            print("    Run: groups | grep video")
            print("    If not in video group: sudo usermod -a -G video $USER")
            print("    Then log out and log back in")
            print("\nQuick test:")
            print("  ls -l /dev/video0  # Check permissions")
            print("  id | grep video    # Check if current session has video group")
            print("\nExamples:")
            print("  python3 yolov8_native_tracking.py")
            print("  VIDEO_SOURCE=1 python3 yolov8_native_tracking.py")
            print("  VIDEO_SOURCE=./video.mp4 python3 yolov8_native_tracking.py")
            sys.exit(1)

        print("✓ Video source opened successfully!\n")

        # Get video properties
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if isinstance(video_source, int) and (width != desired_width or height != desired_height):
            print(f"⚠️ Requested {desired_width}x{desired_height}, camera returned {width}x{height}.")
        print(f"Video properties: {width}x{height} @ {fps} FPS\n")

    # Prepare tracker config tuned for fast motion and ID persistence
    tracker_config = prepare_tracker_config(tracker_type, fps)
    print("Starting tracking...\n")

    paused = False
    frame_count = 0
    locked_track_id = None  # Lock onto specific track ID
    lock_frames_needed = 5   # Frames before locking
    lock_frame_count = 0     # Counter for lock stability
    locked_miss_count = 0    # Consecutive frames where locked ID is missing
    last_locked_bbox = None  # Last locked box for re-association
    last_locked_class = None  # Last locked class for label persistence
    last_locked_mask = None  # Last mask for locked object (helps re-association on angle change)
    # Fallback OpenCV tracker (detect once, then track) for extra persistence
    use_cv_tracker = _str_to_bool(os.environ.get("USE_CV_TRACKER", "true"))
    cv_tracker = None
    cv_tracker_active = False

    # Classification stability tracking
    class_history = {}  # track_id -> list of (class_name, confidence) tuples
    stable_class = {}   # track_id -> stable class name

    # Motion trail tracking for helping with fast motion
    from collections import deque
    track_trails = {}  # track_id -> deque of (cx, cy) positions
    MAX_TRAIL_LENGTH = 30  # Shorter trail - only recent 30 points
    track_velocities = {}  # track_id -> deque of velocity values

    # Initialize orientation and cropper instances
    orientation_calc = OrientationCalculator()
    upright_cropper = UprightCropper()

    # Create second window for upright crop
    CROP_WINDOW = "Upright View"
    cv2.namedWindow(CROP_WINDOW)

    # Object crop management
    import shutil
    # Save crops relative to script location, not current working directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    crops_dir = os.path.join(script_dir, "object_crops")

    # Clean/create crops directory at startup
    if os.path.exists(crops_dir):
        shutil.rmtree(crops_dir)
    os.makedirs(crops_dir, exist_ok=True)

    active_tracks = set()  # Track IDs currently visible
    saved_crops = {}  # track_id -> crop_file_path

    print(f"Object crops will be saved to: {crops_dir}")
    print("Crops are automatically managed (created/deleted) based on tracking\n")

    # Main tracking loop
    while True:
        if not paused:
            # Get frame from either ROS 2 or OpenCV VideoCapture
            if USE_ROS2:
                frame = ros2_node.get_frame()
                if frame is None:
                    print("\nNo frame available from ROS 2 topic.")
                    cv2.waitKey(10)
                    continue
                ret = True
            else:
                ret, frame = cap.read()

                if not ret:
                    print("\nEnd of video or unable to read frame.")
                    break

            frame_count += 1

            # Detect and track objects using YOLOv8's built-in tracker
            # persist=True maintains tracking IDs across frames
            # Optimized for fast-moving objects:
            # - Lower IoU threshold allows matching even when object moves far
            # - Higher max_age keeps tracks alive longer during occlusions
            # - imgsz can be increased for better small object detection
            # EXCLUDE person class (class 0 in COCO dataset)
            results = model.track(
                frame,
                persist=True,
                verbose=False,
                conf=conf_threshold,
                iou=iou_threshold,
                tracker=tracker_config,
                # Maximum image size for best classification accuracy
                imgsz=int(os.environ.get("IMG_SIZE", "1280")),  # Highest resolution for maximum accuracy
                # Exclude person (class 0) - only detect objects
                classes=[i for i in range(1, 80)],  # COCO has 80 classes, skip class 0 (person)
                # Half precision for faster inference
                half=use_half_precision,
                # More aggressive detection - helps with fast motion
                agnostic_nms=False,  # Class-aware NMS
                max_det=int(os.environ.get("MAX_DET", "3")),  # Limit duplicates; tune via env (default 3)
            )

            # Note: Cannot override class labels in PyTorch inference mode
            # The stable_class dictionary is used for display purposes later
            # if results[0].boxes is not None and len(results[0].boxes) > 0 and results[0].boxes.id is not None:
            #     boxes = results[0].boxes
            #     # Cannot modify cls tensor directly - it's read-only during inference
            #     pass

            # Plot results (draws bounding boxes, labels, and tracking IDs)
            annotated_frame = results[0].plot()

            # Remove duplicate/overlapping detections (multi-class NMS edge cases)
            keep_indices = _dedupe_overlapping_boxes(results[0].boxes, locked_track_id, iou_thr=0.6)

            # Draw motion trails for all tracked objects
            if results[0].boxes is not None and len(results[0].boxes) > 0 and results[0].boxes.id is not None:
                boxes = results[0].boxes
                track_ids_list = boxes.id.cpu().numpy().astype(int)

                for idx, tid in enumerate(track_ids_list):
                    if idx not in keep_indices and (locked_track_id is None or tid != locked_track_id):
                        continue
                    # Get bbox center
                    bbox = boxes.xyxy[idx].cpu().numpy()
                    cx = int((bbox[0] + bbox[2]) / 2)
                    cy = int((bbox[1] + bbox[3]) / 2)

                    # Initialize trail for new tracks
                    if tid not in track_trails:
                        track_trails[tid] = deque(maxlen=MAX_TRAIL_LENGTH)
                        track_velocities[tid] = deque(maxlen=10)

                    # Calculate velocity
                    velocity = 0
                    if len(track_trails[tid]) > 0:
                        prev_cx, prev_cy = track_trails[tid][-1]
                        velocity = math.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                        track_velocities[tid].append(velocity)

                    # Add current position
                    track_trails[tid].append((cx, cy))

                    # Draw trail
                    if len(track_trails[tid]) > 1:
                        points = np.array(track_trails[tid], dtype=np.int32)

                        # Compute average velocity
                        avg_vel = np.mean(track_velocities[tid]) if track_velocities[tid] else 0

                        # Much thicker trail - more visible
                        base_thickness = 5 if avg_vel < 10 else 7 if avg_vel < 30 else 10

                        # Color: Bright green for locked, Cyan for unlocked
                        trail_color = (0, 255, 0) if tid == locked_track_id else (255, 200, 0)

                        # Draw trail lines with fade effect - THICKER
                        for i in range(1, len(points)):
                            alpha = i / len(points)
                            thickness = max(3, int(base_thickness * alpha))  # Minimum 3px
                            cv2.line(annotated_frame, tuple(points[i-1]), tuple(points[i]),
                                   trail_color, thickness, cv2.LINE_AA)

                        # Draw larger circles at intervals
                        for i, point in enumerate(points[::3]):  # Every 3rd point (more circles)
                            alpha = (i * 3) / len(points)
                            radius = max(3, int(6 * alpha))  # Larger circles
                            cv2.circle(annotated_frame, tuple(point), radius, trail_color, -1)

                # Clean up trails for lost tracks
                active_track_ids = set(track_ids_list)
                trails_to_remove = [tid for tid in track_trails.keys() if tid not in active_track_ids]
                for tid in trails_to_remove:
                    del track_trails[tid]
                    if tid in track_velocities:
                        del track_velocities[tid]

                # Save crops for each detected object and manage cleanup
                current_frame_tracks = set()

                for idx, tid in enumerate(track_ids_list):
                    if idx not in keep_indices:
                        continue

                    current_frame_tracks.add(tid)

                    # Get object bbox and class
                    bbox = boxes.xyxy[idx].cpu().numpy()
                    x1, y1, x2, y2 = map(int, bbox)
                    cls_id = int(boxes.cls[idx].cpu().numpy())
                    obj_name = model.names[cls_id]

                    # Use stable class if available
                    if tid in stable_class:
                        obj_name = stable_class[tid]

                    # Extract crop from frame
                    crop = frame[max(0, y1):min(frame.shape[0], y2),
                                max(0, x1):min(frame.shape[1], x2)]

                    if crop.size > 0:
                        # Save crop with track ID and class name
                        crop_filename = f"track_{tid:04d}_{obj_name.replace(' ', '_')}.jpg"
                        crop_path = os.path.join(crops_dir, crop_filename)

                        # Check if this is a new track
                        is_new_track = tid not in saved_crops

                        # Save or update crop
                        cv2.imwrite(crop_path, crop)
                        saved_crops[tid] = crop_path

                        # Print message for new crops
                        if is_new_track:
                            print(f"💾 Saved new crop: {crop_filename}")

                # Remove crops for objects no longer visible
                lost_tracks = active_tracks - current_frame_tracks
                for tid in lost_tracks:
                    if tid in saved_crops:
                        crop_path = saved_crops[tid]
                        if os.path.exists(crop_path):
                            os.remove(crop_path)
                            print(f"🗑️  Removed crop for lost track ID {tid}")
                        del saved_crops[tid]

                # Update active tracks
                active_tracks = current_frame_tracks

            # Add frame counter and crop count
            cv2.putText(
                annotated_frame,
                f"Frame: {frame_count} | Crops: {len(saved_crops)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Display tracking statistics and create upright crop
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                num_objects = len(results[0].boxes)
                boxes = results[0].boxes

                # Debug: Print detected objects every 30 frames
                if frame_count % 30 == 0:
                    print(f"\n=== Frame {frame_count} - Detected objects ===")
                    for idx in range(len(boxes)):
                        if idx not in keep_indices:
                            continue
                        cls_id = int(boxes.cls[idx].cpu().numpy())
                        conf = float(boxes.conf[idx].cpu().numpy())
                        obj_name = model.names[cls_id]
                        bbox = boxes.xyxy[idx].cpu().numpy()
                        width = bbox[2] - bbox[0]
                        height = bbox[3] - bbox[1]
                        area = width * height

                        # Show stable classification if available
                        if boxes.id is not None:
                            tid = int(boxes.id[idx].cpu().numpy())
                            if tid in stable_class:
                                stable_name = stable_class[tid]
                                if stable_name != obj_name:
                                    print(f"  - ID {tid}: {obj_name} ({conf:.3f}) → FILTERED, Stable: {stable_name} | {int(width)}x{int(height)}")
                                else:
                                    print(f"  - ID {tid}: {obj_name} ({conf:.3f}) ✓ STABLE | {int(width)}x{int(height)}")
                            else:
                                print(f"  - ID {tid}: {obj_name} ({conf:.3f}) [Learning...] | {int(width)}x{int(height)}")
                        else:
                            print(f"  - {obj_name}: {conf:.3f} | Size: {int(width)}x{int(height)} ({int(area)} px²)")

                # Update classification history for stability filtering
                if boxes.id is not None:
                    for idx in range(len(boxes)):
                        if idx not in keep_indices:
                            continue
                        tid = int(boxes.id[idx].cpu().numpy())
                        cls_id = int(boxes.cls[idx].cpu().numpy())
                        conf = float(boxes.conf[idx].cpu().numpy())
                        obj_name = model.names[cls_id]

                        # Get object dimensions
                        bbox = boxes.xyxy[idx].cpu().numpy()
                        width = bbox[2] - bbox[0]
                        height = bbox[3] - bbox[1]

                        # AGGRESSIVE OVERRIDES for phone mislabels (remote/knife/donut/etc.)
                        # Rule: if dimensions look like a phone, force to phone even if class differs.
                        phone_valid, _ = ObjectValidator.is_valid_classification("cell phone", width, height)
                        phone_like_shape = phone_valid or (width * height > 15000 and (width / max(height, 1)) > 1.4)
                        if phone_like_shape and obj_name in {"remote", "knife", "donut", "spoon", "fork", "toothbrush"}:
                            obj_name = "cell phone"
                            if frame_count % 30 == 0:
                                area_dbg = int(width * height)
                                aspect_dbg = width / max(height, 1)
                                print(
                                    f"  🔄 ID {tid}: {model.names[cls_id]} → cell phone OVERRIDE "
                                    f"(area={area_dbg}, aspect={aspect_dbg:.2f})"
                                )

                        # AGGRESSIVE OVERRIDE: Toothbrush → Phone misclassification prevention
                        # If classified as toothbrush but size/aspect fit phone better, override immediately
                        area = width * height
                        aspect_ratio = width / height if height > 0 else 0

                        if obj_name == "toothbrush":
                            toothbrush_valid, _ = ObjectValidator.is_valid_classification("toothbrush", width, height)

                            # Override when it matches phone AND either it is implausible for a toothbrush
                            # or it is simply larger/wider than a toothbrush would be.
                            if phone_valid and (not toothbrush_valid or area > 1500 or aspect_ratio > 0.5):
                                obj_name = "cell phone"
                                if frame_count % 30 == 0:
                                    print(
                                        f"  🔄 ID {tid}: toothbrush → cell phone OVERRIDE "
                                        f"(area={int(area)}, aspect={aspect_ratio:.2f})"
                                    )

                        # Validate classification based on physical characteristics
                        is_valid, reason = ObjectValidator.is_valid_classification(obj_name, width, height)

                        # Only add to history if physically plausible
                        if tid not in class_history:
                            class_history[tid] = []

                        if is_valid:
                            class_history[tid].append((obj_name, conf))
                        else:
                            # Rejected - try to infer correct class from size
                            plausible = ObjectValidator.get_plausible_classes(width, height)
                            if plausible:
                                # Use the most plausible class (highest priority)
                                inferred_class = plausible[0][0]
                                # Add to history with slightly reduced confidence
                                class_history[tid].append((inferred_class, conf * 0.9))

                                if frame_count % 30 == 0:
                                    plausible_names = ", ".join([name for name, _ in plausible[:3]])
                                    print(f"  ⚠️  ID {tid}: {obj_name} REJECTED ({reason}) | Inferred: {inferred_class} | Alternatives: {plausible_names}")
                            else:
                                # No plausible alternatives - skip this detection
                                if frame_count % 30 == 0:
                                    print(f"  ⚠️  ID {tid}: {obj_name} REJECTED ({reason}) | No plausible alternatives")

                        if len(class_history[tid]) > 10:
                            class_history[tid].pop(0)

                        # Determine stable classification (most common in last 10 frames)
                        # Only if we have enough valid samples - FASTER CONVERGENCE
                        if len(class_history[tid]) >= 2:  # Reduced from 3 to 2 for even faster convergence
                            class_counts = {}
                            for cls_name, _ in class_history[tid]:
                                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
                            # Stable class is the most common one
                            stable_class[tid] = max(class_counts, key=class_counts.get)

                # Count unique IDs
                track_ids = set()
                if boxes.id is not None:
                    track_ids = set(boxes.id.cpu().numpy().astype(int))

                cv2.putText(
                    annotated_frame,
                    f"Objects: {num_objects} | IDs: {len(track_ids)}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                # Show tracking persistence info
                cv2.putText(
                    annotated_frame,
                    f"Conf: {conf_threshold:.2f} | IoU: {iou_threshold:.2f}",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                )

                # Create upright crop - LOCK onto one specific track ID
                if len(boxes) > 0 and boxes.id is not None:
                    # Find which object to track
                    best_idx = None
                    track_ids_list = boxes.id.cpu().numpy().astype(int)
                    valid_indices = [i for i in range(len(boxes)) if i in keep_indices or locked_track_id is not None and track_ids_list[i] == locked_track_id]
                    if not valid_indices:
                        valid_indices = list(range(len(boxes)))  # fallback

                    # If we have a locked ID, try to find it
                    if locked_track_id is not None:
                        for idx, tid in enumerate(track_ids_list):
                            if tid == locked_track_id:
                                best_idx = idx
                                lock_frame_count += 1
                                locked_miss_count = 0
                                # If YOLO lock is good again, stop CV fallback
                                if cv_tracker_active:
                                    cv_tracker_active = False
                                    cv_tracker = None
                                break

                        # Lost locked track - reset after many frames
                        if best_idx is None:
                            # Try to re-associate with nearby box (ID flip when angle changes)
                            reassoc_idx = None
                            best_iou = 0.0
                            if last_locked_bbox is not None:
                                for idx, tid in enumerate(track_ids_list):
                                    candidate_box = boxes.xyxy[idx].cpu().numpy()
                                    iou = _bbox_iou(candidate_box, last_locked_bbox)
                                    cx_prev = (last_locked_bbox[0] + last_locked_bbox[2]) / 2
                                    cy_prev = (last_locked_bbox[1] + last_locked_bbox[3]) / 2
                                    cx_new = (candidate_box[0] + candidate_box[2]) / 2
                                    cy_new = (candidate_box[1] + candidate_box[3]) / 2
                                    center_dist = math.hypot(cx_new - cx_prev, cy_new - cy_prev)
                                    if (iou > 0.15 or center_dist < 120) and iou >= best_iou:
                                        best_iou = iou
                                        reassoc_idx = idx

                            # Secondary attempt: mask IoU if segmentation is available
                            if reassoc_idx is None and last_locked_mask is not None and results[0].masks is not None:
                                best_mask_iou = 0.0
                                for idx, tid in enumerate(track_ids_list):
                                    if idx >= len(results[0].masks.data):
                                        continue
                                    candidate_mask = results[0].masks.data[idx].cpu().numpy().astype(bool)
                                    miou = _mask_iou(candidate_mask, last_locked_mask)
                                    if miou > 0.1 and miou > best_mask_iou:
                                        best_mask_iou = miou
                                        reassoc_idx = idx
                                if reassoc_idx is not None:
                                    best_iou = best_mask_iou

                            if reassoc_idx is not None:
                                new_tid = int(track_ids_list[reassoc_idx])
                                if last_locked_class is not None:
                                    stable_class[new_tid] = last_locked_class
                                locked_track_id = new_tid
                                best_idx = reassoc_idx
                                lock_frame_count = 0
                                locked_miss_count = 0
                                if frame_count % 30 == 0:
                                    print(f"  🔄 Re-locked to new ID {new_tid} via IoU {best_iou:.2f}")
                            else:
                                locked_miss_count += 1
                                lock_frame_count = 0
                                # Start CV tracker fallback once when we start missing the lock
                                if (
                                    use_cv_tracker
                                    and not cv_tracker_active
                                    and last_locked_bbox is not None
                                ):
                                    cv_tracker = _create_cv_tracker()
                                    if cv_tracker is not None:
                                        x1, y1, x2, y2 = [int(v) for v in last_locked_bbox]
                                        w, h = x2 - x1, y2 - y1
                                        if w > 2 and h > 2:
                                            try:
                                                cv_tracker.init(frame, (x1, y1, w, h))
                                                cv_tracker_active = True
                                                if frame_count % 30 == 0:
                                                    print("  🔄 CV tracker fallback started (OpenCV) to hold lock")
                                            except Exception as exc:
                                                cv_tracker = None
                                                cv_tracker_active = False
                                                if frame_count % 30 == 0:
                                                    print(f"  ⚠️ Could not start CV tracker: {exc}")
                                    if locked_miss_count > 15:
                                        if frame_count % 30 == 0:  # Check every second at 30fps
                                            print(f"Lost locked track ID {locked_track_id} for {locked_miss_count} frames, releasing lock...")
                                        locked_track_id = None
                                        locked_miss_count = 0

                    # If no locked ID or lost, select object with highest motion
                    if locked_track_id is None and boxes.conf is not None:
                        # Select object with highest average velocity (most movement)
                        max_velocity = 0
                        best_idx = None

                        for idx, tid in enumerate(track_ids_list):
                            if idx not in valid_indices:
                                continue
                            if tid in track_velocities and len(track_velocities[tid]) > 0:
                                avg_vel = np.mean(track_velocities[tid])
                                if avg_vel > max_velocity:
                                    max_velocity = avg_vel
                                    best_idx = idx

                        # If no moving object found, fallback to highest confidence
                        if best_idx is None:
                            # fallback to highest confidence among valid indices
                            best_idx = max(valid_indices, key=lambda j: boxes.conf[j].item())

                        candidate_id = int(track_ids_list[best_idx])

                        # Lock onto this ID after seeing it for several frames
                        if lock_frame_count >= lock_frames_needed:
                            # If this overlaps the last locked box, carry over its class to avoid relabeling
                            if last_locked_bbox is not None:
                                candidate_box = boxes.xyxy[best_idx].cpu().numpy()
                                iou = _bbox_iou(candidate_box, last_locked_bbox)
                                cx_prev = (last_locked_bbox[0] + last_locked_bbox[2]) / 2
                                cy_prev = (last_locked_bbox[1] + last_locked_bbox[3]) / 2
                                cx_new = (candidate_box[0] + candidate_box[2]) / 2
                                cy_new = (candidate_box[1] + candidate_box[3]) / 2
                                center_dist = math.hypot(cx_new - cx_prev, cy_new - cy_prev)
                                if iou > 0.35 or center_dist < 80:
                                    if last_locked_class is not None:
                                        stable_class[candidate_id] = last_locked_class
                                        if frame_count % 30 == 0:
                                            print(f"  🔄 Re-associated class to new ID {candidate_id}: {last_locked_class} (IoU {iou:.2f}, dist {center_dist:.1f})")

                            locked_track_id = candidate_id
                            motion_info = f" (velocity: {max_velocity:.1f}px/f)" if max_velocity > 0 else ""
                            print(f"✓ Locked onto MOVING track ID: {locked_track_id} ({model.names[int(boxes.cls[best_idx])]}){motion_info}")
                            lock_frame_count = 0
                            cv_tracker_active = False
                            cv_tracker = None
                        else:
                            lock_frame_count += 1

                    # Process the selected object
                    if best_idx is not None:
                        box = boxes.xyxy[best_idx].cpu().numpy()
                        cls_id = int(boxes.cls[best_idx].cpu().numpy())
                        obj_name = model.names[cls_id]
                        track_id = int(track_ids_list[best_idx])

                        # Use stable classification if available, otherwise try to infer from size
                        if track_id in stable_class:
                            obj_name = stable_class[track_id]
                        else:
                            # No stable class yet - check if current detection is invalid
                            width = box[2] - box[0]
                            height = box[3] - box[1]
                            is_valid, reason = ObjectValidator.is_valid_classification(obj_name, width, height)

                            if not is_valid:
                                # Current class is invalid - try to suggest a plausible one
                                plausible = ObjectValidator.get_plausible_classes(width, height)
                                if plausible:
                                    # Use the first plausible class as a hint
                                    obj_name = plausible[0][0] + "?"  # Add ? to show it's uncertain

                        # Calculate orientation using PCA method
                        angle = None
                        cx = cy = None
                        if results[0].masks is not None and len(results[0].masks.data) > best_idx:
                            mask = results[0].masks.data[best_idx].cpu().numpy()
                            angle, cx, cy = orientation_calc.calculate(mask, frame.shape)

                            # Draw orientation arrow on main frame
                            if angle is not None and cx is not None:
                                cx_int, cy_int = int(cx), int(cy)
                                cv2.circle(annotated_frame, (cx_int, cy_int), 10, (0, 0, 255), -1)
                                rad = np.radians(angle + 90)
                                dx = int(80 * np.cos(rad))
                                dy = int(80 * np.sin(rad))
                                cv2.arrowedLine(annotated_frame, (cx_int, cy_int),
                                               (cx_int + dx, cy_int + dy),
                                               (255, 0, 255), 5, tipLength=0.3)

                        # Create and show upright crop
                        crop = upright_cropper.create_crop(frame, box, angle, obj_name, track_id)
                        last_locked_bbox = box
                        last_locked_class = obj_name
                        if results[0].masks is not None and len(results[0].masks.data) > best_idx:
                            last_locked_mask = results[0].masks.data[best_idx].cpu().numpy().astype(bool)
                        if crop is not None:
                            cv2.imshow(CROP_WINDOW, crop)
            else:
                # No YOLO detections; try CV tracker fallback
                if cv_tracker_active and cv_tracker is not None:
                    success, bbox = cv_tracker.update(frame)
                    if success:
                        x, y, w, h = map(int, bbox)
                        last_locked_bbox = np.array([x, y, x + w, y + h])
                        cx, cy = x + w // 2, y + h // 2
                        last_locked_class = last_locked_class or "object"
                        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 200, 255), 3)
                        cv2.putText(
                            annotated_frame,
                            f"{last_locked_class} [CV Tracker]",
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 200, 255),
                            2,
                        )
                        cv2.circle(annotated_frame, (cx, cy), 6, (0, 0, 255), -1)
                    else:
                        cv_tracker_active = False
                        cv_tracker = None
                        if frame_count % 30 == 0:
                            print("CV tracker lost target. Waiting for YOLO re-detect.")
                else:
                    orientation_calc.reset()
                    lock_frame_count = 0
                    # Show "no objects detected" message
                    if frame_count % 30 == 0:
                        print(f"\n=== Frame {frame_count} - No objects detected ===")
                        print("  Tip: Try lowering confidence with '-' key or adjust lighting")

            # Show lock status on main window
            lock_status = f"LOCKED: ID {locked_track_id}" if locked_track_id is not None else "UNLOCKED (auto-selecting)"
            lock_color = (0, 255, 0) if locked_track_id is not None else (0, 165, 255)
            cv2.putText(
                annotated_frame,
                lock_status,
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                lock_color,
                2,
            )

            # Visualize main tracking window
            cv2.imshow('YOLOv8 Object Tracking', annotated_frame)
        else:
            # Show paused message
            paused_frame = frame.copy()
            cv2.putText(
                paused_frame,
                "PAUSED - Press 'p' to resume",
                (50, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
            )
            cv2.imshow('YOLOv8 Object Tracking', paused_frame)

        # Handle keyboard input
        key = cv2.waitKey(25) & 0xFF
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused" if paused else "Resumed")
        elif key == ord('+') or key == ord('='):
            conf_threshold = min(0.9, conf_threshold + 0.05)
            print(f"Confidence threshold: {conf_threshold:.2f}")
        elif key == ord('-') or key == ord('_'):
            conf_threshold = max(0.1, conf_threshold - 0.05)
            print(f"Confidence threshold: {conf_threshold:.2f}")
        elif key == ord('l'):
            # Toggle lock - release current lock to select new object
            if locked_track_id is not None:
                print(f"Released lock on ID {locked_track_id}")
                locked_track_id = None
                lock_frame_count = 0
            else:
                print("Already unlocked - will auto-lock on next stable object")
        elif key == ord('r'):
            # Reset everything including trails
            locked_track_id = None
            lock_frame_count = 0
            last_locked_bbox = None
            last_locked_class = None
            last_locked_mask = None
            orientation_calc.reset()
            track_trails.clear()
            track_velocities.clear()
            cv_tracker_active = False
            cv_tracker = None
            print("Reset tracking state and cleared all motion trails")

    # Cleanup
    if cap is not None:
        cap.release()
    if USE_ROS2 and ros2_node is not None:
        ros2_node.destroy_node()
        rclpy.shutdown()
        print("\n✓ ROS 2 shutdown complete.")
    cv2.destroyAllWindows()

    # Summary of saved crops
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Processed {frame_count} frames.")
    print(f"Final crops saved: {len(saved_crops)}")
    if saved_crops:
        print(f"\nCrops location: {crops_dir}")
        print("Saved crops:")
        for tid, path in sorted(saved_crops.items()):
            filename = os.path.basename(path)
            print(f"  - {filename}")
    else:
        print("\nNo crops remain (all objects lost tracking)")
    print(f"{'='*60}")
    print("Done!")


if __name__ == "__main__":
    # Check if help is requested
    if len(sys.argv) > 1 and sys.argv[1] in ["-h", "--help"]:
        print("YOLOv8 Native Object Tracking")
        print("\nUsage:")
        print("  python3 yolov8_native_tracking.py")
        print("\nEnvironment Variables:")
        print("  USE_ROS2 - Enable ROS 2 mode (default: TRUE)")
        print("             Set to 'false' to use camera/video instead")
        print("  ROS2_TOPIC - ROS 2 camera topic (default: /camera/color/image_raw)")
        print("               Only used when USE_ROS2=true")
        print("  YOLO_MODEL - Model to use (default: auto-detect, prefers YOLOv11 if present)")
        print("               Segmentation models (with orientation): yolov11n-seg.pt, yolov11s-seg.pt, yolov8x-seg.pt, yolov8n-seg.pt")
        print("               Detection models (no orientation): yolov11n.pt, yolov11s.pt, yolov8n.pt, yolov8s.pt")
        print("  VIDEO_SOURCE - Video source (default: 0)")
        print("                 Only used when USE_ROS2=false")
        print("                 0, 1, 2... for webcam, or path to video file")
        print("  CONF_THRESHOLD - Detection confidence (default: 0.20)")
        print("                   Lower = more detections, higher = fewer false positives")
        print("  IOU_THRESHOLD - Tracking IoU threshold (default: 0.05)")
        print("                  Lower = handles fast motion better")
        print("  MAX_AGE - Frames to keep lost tracks (default: 200)")
        print("            Higher = maintains ID longer when object temporarily lost")
        print("  MIN_HITS - Min detections before confirmed (default: 1)")
        print("             Lower = faster track confirmation")
        print("  TRACKER - Tracker type (default: botsort_aggressive.yaml)")
        print("            Options: bytetrack.yaml, botsort.yaml, botsort_aggressive.yaml")
        print("\nExamples:")
        print("  # ROS 2 with RealSense (DEFAULT - implicit USE_ROS2=true)")
        print("  python3 yolov8_native_tracking.py")
        print("\n  # ROS 2 with Gazebo camera")
        print("  ROS2_TOPIC=/camera python3 yolov8_native_tracking.py")
        print("\n  # Use camera laptop (disable ROS 2)")
        print("  USE_ROS2=false python3 yolov8_native_tracking.py")
        print("\n  # Use different laptop camera")
        print("  USE_ROS2=false VIDEO_SOURCE=1 python3 yolov8_native_tracking.py")
        print("\n  # Use video file")
        print("  USE_ROS2=false VIDEO_SOURCE=./test.mp4 python3 yolov8_native_tracking.py")
        print("\n  # Use YOLOv11 segmentation model if downloaded locally")
        print("  YOLO_MODEL=yolov11n-seg.pt python3 yolov8_native_tracking.py")
        print("\n  # Use larger model for better detection (YOLOv8 example)")
        print("  YOLO_MODEL=yolov8x-seg.pt python3 yolov8_native_tracking.py")
        print("\n  # Optimized for FAST-MOVING objects (recommended)")
        print("  CONF_THRESHOLD=0.2 IOU_THRESHOLD=0.05 MAX_AGE=200 python3 yolov8_native_tracking.py")
        print("\n  # Use ByteTrack tracker explicitly")
        print("  TRACKER=bytetrack.yaml python3 yolov8_native_tracking.py")
        print("\n  # Use BoT-SORT tracker (better for crowded scenes)")
        print("  TRACKER=botsort_aggressive.yaml python3 yolov8_native_tracking.py")
        print("\n  # Maximum persistence (for very fast objects)")
        print("  CONF_THRESHOLD=0.15 IOU_THRESHOLD=0.03 MAX_AGE=200 python3 yolov8_native_tracking.py")
        print("\n  # ROS 2 with Gazebo camera")
        print("  USE_ROS2=true ROS2_TOPIC=/camera python3 yolov8_native_tracking.py")
        sys.exit(0)

    main()
