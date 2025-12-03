#!/usr/bin/env python3
"""
Object Tracking with Rotation Detection
Uses ORB features to track object including rotation and scale changes.
"""

import cv2
import numpy as np
import math

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class RotationTracker:
    """Tracker with rotation detection using ORB features."""
    
    def __init__(self, model_name='yolov8n.pt', confidence=0.3):
        self.model = YOLO(model_name)
        self.confidence = confidence
        
        # Trail
        self.pts = []
        self.max_pts = 100
        
        # Template and features
        self.template = None
        self.template_kp = None
        self.template_desc = None
        self.template_center = None
        self.last_bbox = None
        self.tracked_class = None
        self.is_tracking = False
        
        # Rotation tracking
        self.current_rotation = 0  # degrees
        self.current_scale = 1.0
        
        # ORB detector
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Search region
        self.search_margin = 100
        
        # YOLO refresh
        self.frames_since_yolo = 0
        self.yolo_interval = 90
        
        # Target objects
        self.target_classes = ['bottle', 'cup', 'cell phone', 'remote', 
                               'mouse', 'keyboard', 'book', 'scissors',
                               'teddy bear', 'vase', 'apple', 'orange', 'banana',
                               'wine glass', 'fork', 'knife', 'spoon', 'bowl']
    
    def detect_with_yolo(self, frame):
        """Run YOLO to find an object."""
        results = self.model(frame, conf=self.confidence, verbose=False)
        
        best = None
        best_area = 0
        
        for result in results:
            for box in result.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                
                if cls_name in self.target_classes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    
                    if area > best_area:
                        best_area = area
                        best = (x1, y1, x2, y2, cls_name)
        
        return best
    
    def extract_template(self, frame, bbox):
        """Extract template and compute ORB features."""
        x1, y1, x2, y2 = bbox[:4]
        
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        if x2 <= x1 or y2 <= y1:
            return False
        
        self.template = frame[y1:y2, x1:x2].copy()
        
        # Compute ORB features on template
        gray_template = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        self.template_kp, self.template_desc = self.orb.detectAndCompute(gray_template, None)
        
        if self.template_desc is None or len(self.template_kp) < 4:
            # Not enough features, use template matching fallback
            self.template_kp = None
            self.template_desc = None
        
        th, tw = self.template.shape[:2]
        self.template_center = (tw / 2, th / 2)
        self.last_bbox = (x1, y1, x2, y2)
        self.current_rotation = 0
        self.current_scale = 1.0
        
        return True
    
    def get_search_region(self, frame):
        """Get expanded search region."""
        if self.last_bbox is None:
            return frame, 0, 0
        
        x1, y1, x2, y2 = self.last_bbox
        h, w = frame.shape[:2]
        
        # Expand based on scale
        margin = int(self.search_margin * max(1, self.current_scale))
        
        sx1 = max(0, x1 - margin)
        sy1 = max(0, y1 - margin)
        sx2 = min(w, x2 + margin)
        sy2 = min(h, y2 + margin)
        
        return frame[sy1:sy2, sx1:sx2], sx1, sy1
    
    def match_features(self, frame):
        """Match ORB features to find object with rotation."""
        if self.template_desc is None or len(self.template_kp) < 4:
            return None
        
        # Get search region
        search_region, offset_x, offset_y = self.get_search_region(frame)
        
        if search_region.size == 0:
            return None
        
        # Compute features in search region
        gray_search = cv2.cvtColor(search_region, cv2.COLOR_BGR2GRAY)
        kp_search, desc_search = self.orb.detectAndCompute(gray_search, None)
        
        if desc_search is None or len(kp_search) < 4:
            return None
        
        # Match features
        try:
            matches = self.bf.knnMatch(self.template_desc, desc_search, k=2)
        except:
            return None
        
        # Apply ratio test
        good_matches = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
        
        if len(good_matches) < 4:
            return None
        
        # Get matched keypoints
        src_pts = np.float32([self.template_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_search[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        # Find homography
        try:
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        except:
            return None
        
        if M is None:
            return None
        
        # Check if homography is valid
        inliers = mask.ravel().sum() if mask is not None else 0
        if inliers < 4:
            return None
        
        # Get template corners
        th, tw = self.template.shape[:2]
        corners = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]]).reshape(-1, 1, 2)
        
        # Transform corners
        try:
            transformed = cv2.perspectiveTransform(corners, M)
        except:
            return None
        
        # Get bounding rect and rotation
        transformed = transformed.reshape(-1, 2)
        
        # Calculate rotation from the homography
        rotation = -math.atan2(M[0, 1], M[0, 0]) * 180 / math.pi
        
        # Calculate scale
        scale_x = math.sqrt(M[0, 0]**2 + M[0, 1]**2)
        scale_y = math.sqrt(M[1, 0]**2 + M[1, 1]**2)
        scale = (scale_x + scale_y) / 2
        
        # Validate scale
        if scale < 0.3 or scale > 3.0:
            return None
        
        # Get bounding box
        x_min = int(transformed[:, 0].min()) + offset_x
        y_min = int(transformed[:, 1].min()) + offset_y
        x_max = int(transformed[:, 0].max()) + offset_x
        y_max = int(transformed[:, 1].max()) + offset_y
        
        # Get center
        center_x = int(transformed[:, 0].mean()) + offset_x
        center_y = int(transformed[:, 1].mean()) + offset_y
        
        # Get rotated corners in full frame coords
        rotated_corners = transformed + np.array([offset_x, offset_y])
        
        confidence = inliers / len(good_matches)
        
        return {
            'bbox': (x_min, y_min, x_max, y_max),
            'center': (center_x, center_y),
            'rotation': rotation,
            'scale': scale,
            'corners': rotated_corners.astype(np.int32),
            'confidence': confidence,
            'matches': len(good_matches),
            'inliers': inliers
        }
    
    def match_template_fallback(self, frame):
        """Fallback to template matching without rotation."""
        if self.template is None:
            return None
        
        search_region, offset_x, offset_y = self.get_search_region(frame)
        
        th, tw = self.template.shape[:2]
        sh, sw = search_region.shape[:2]
        
        if tw >= sw or th >= sh:
            return None
        
        result = cv2.matchTemplate(search_region, self.template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        
        if max_val < 0.4:
            return None
        
        x1 = max_loc[0] + offset_x
        y1 = max_loc[1] + offset_y
        x2 = x1 + tw
        y2 = y1 + th
        
        return {
            'bbox': (x1, y1, x2, y2),
            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            'rotation': 0,
            'scale': 1.0,
            'corners': None,
            'confidence': max_val,
            'matches': 0,
            'inliers': 0
        }
    
    def update(self, frame):
        """Main update with rotation tracking."""
        
        self.frames_since_yolo += 1
        
        # Periodic YOLO refresh
        if self.is_tracking and self.frames_since_yolo >= self.yolo_interval:
            detection = self.detect_with_yolo(frame)
            if detection:
                x1, y1, x2, y2, cls_name = detection
                self.extract_template(frame, detection)
                self.tracked_class = cls_name
                self.frames_since_yolo = 0
                
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                self.pts.append(center)
                if len(self.pts) > self.max_pts:
                    self.pts.pop(0)
                
                return {
                    'bbox': (x1, y1, x2, y2),
                    'center': center,
                    'class': cls_name,
                    'status': 'YOLO Update',
                    'rotation': 0,
                    'scale': 1.0,
                    'corners': None,
                    'confidence': 1.0
                }
        
        # Try feature matching first
        if self.is_tracking:
            match = self.match_features(frame)
            
            if match and match['confidence'] > 0.3:
                x1, y1, x2, y2 = match['bbox']
                self.last_bbox = (x1, y1, x2, y2)
                self.current_rotation = match['rotation']
                self.current_scale = match['scale']
                
                self.pts.append(match['center'])
                if len(self.pts) > self.max_pts:
                    self.pts.pop(0)
                
                return {
                    'bbox': match['bbox'],
                    'center': match['center'],
                    'class': self.tracked_class,
                    'status': 'Features',
                    'rotation': match['rotation'],
                    'scale': match['scale'],
                    'corners': match['corners'],
                    'confidence': match['confidence']
                }
            
            # Fallback to template matching
            match = self.match_template_fallback(frame)
            if match:
                x1, y1, x2, y2 = match['bbox']
                self.last_bbox = (x1, y1, x2, y2)
                
                self.pts.append(match['center'])
                if len(self.pts) > self.max_pts:
                    self.pts.pop(0)
                
                return {
                    'bbox': match['bbox'],
                    'center': match['center'],
                    'class': self.tracked_class,
                    'status': 'Template',
                    'rotation': self.current_rotation,
                    'scale': self.current_scale,
                    'corners': None,
                    'confidence': match['confidence']
                }
            
            print("Lost tracking, re-detecting...")
            self.is_tracking = False
        
        # YOLO detection
        detection = self.detect_with_yolo(frame)
        
        if detection:
            x1, y1, x2, y2, cls_name = detection
            self.extract_template(frame, detection)
            self.tracked_class = cls_name
            self.is_tracking = True
            self.frames_since_yolo = 0
            
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            self.pts.append(center)
            if len(self.pts) > self.max_pts:
                self.pts.pop(0)
            
            print(f"Detected: {cls_name}")
            return {
                'bbox': (x1, y1, x2, y2),
                'center': center,
                'class': cls_name,
                'status': 'Detected',
                'rotation': 0,
                'scale': 1.0,
                'corners': None,
                'confidence': 1.0
            }
        
        return None
    
    def reset(self):
        """Reset tracker."""
        self.template = None
        self.template_kp = None
        self.template_desc = None
        self.last_bbox = None
        self.tracked_class = None
        self.is_tracking = False
        self.current_rotation = 0
        self.current_scale = 1.0
        self.pts = []
        self.frames_since_yolo = 0
        print("Reset - will re-detect")
    
    def draw(self, frame, detection):
        """Draw with rotation indicator."""
        if detection:
            x1, y1, x2, y2 = detection['bbox']
            center = detection['center']
            status = detection['status']
            rotation = detection.get('rotation', 0)
            scale = detection.get('scale', 1.0)
            corners = detection.get('corners')
            conf = detection.get('confidence', 1.0)
            
            # Colors
            colors = {
                'Detected': (0, 255, 0),
                'Features': (255, 200, 0),
                'Template': (255, 100, 100),
                'YOLO Update': (0, 255, 255)
            }
            color = colors.get(status, (255, 200, 0))
            
            # Draw rotated bounding box if available
            if corners is not None:
                cv2.polylines(frame, [corners], True, color, 2)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center and rotation arrow
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            
            # Rotation arrow
            arrow_len = 40
            angle_rad = math.radians(rotation)
            arrow_end = (
                int(center[0] + arrow_len * math.cos(angle_rad)),
                int(center[1] - arrow_len * math.sin(angle_rad))
            )
            cv2.arrowedLine(frame, center, arrow_end, (0, 255, 255), 2, tipLength=0.3)
            
            # Label
            label = f"{detection['class']} [{status}]"
            cv2.putText(frame, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Trail
            if len(self.pts) > 1:
                pts_array = np.array(self.pts, dtype=np.int32)
                cv2.polylines(frame, [pts_array], False, (0, 0, 255), 2, cv2.LINE_AA)
            
            # Info panel
            cv2.putText(frame, f"Position: {center}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Rotation: {rotation:.1f} deg", (10, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Scale: {scale:.2f}x", (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
            cv2.putText(frame, f"Confidence: {conf:.2f}", (10, 105),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            
            # Show template
            if self.template is not None:
                th, tw = self.template.shape[:2]
                scale_disp = min(80 / tw, 80 / th)
                dw, dh = int(tw * scale_disp), int(th * scale_disp)
                if dw > 0 and dh > 0:
                    template_small = cv2.resize(self.template, (dw, dh))
                    frame[10:10+dh, frame.shape[1]-dw-10:frame.shape[1]-10] = template_small
                    cv2.rectangle(frame, (frame.shape[1]-dw-12, 8),
                                 (frame.shape[1]-8, 12+dh), (255,255,255), 1)
        else:
            cv2.putText(frame, "Searching for object...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return frame


def main():
    print("=" * 50)
    print("Rotation-Aware Object Tracker")
    print("=" * 50)
    
    if not YOLO_AVAILABLE:
        print("\nError: YOLO not found!")
        print("Install with: pip install ultralytics")
        return
    
    print("\nFeatures:")
    print("  - YOLO detection + ORB feature tracking")
    print("  - Rotation detection (yellow arrow)")
    print("  - Scale tracking")
    print("  - Rotated bounding box")
    print("\nControls:")
    print("  q - Quit")
    print("  r - Reset (force re-detection)")
    print("=" * 50)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    tracker = RotationTracker()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.flip(frame, 1)
        detection = tracker.update(frame)
        frame = tracker.draw(frame, detection)
        
        # Status
        status = "Tracking" if tracker.is_tracking else "Detecting"
        cv2.putText(frame, f"Mode: {status}", (10, frame.shape[0] - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, "Press 'q' quit, 'r' reset", (10, frame.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("Object Tracking", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            tracker.reset()
    
    cap.release()
    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == "__main__":
    main()
