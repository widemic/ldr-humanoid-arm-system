#!/usr/bin/env python3
"""
Perfect Object Detection using SAM (Segment Anything Model)
Best segmentation quality - works on ANY object
"""

import cv2
import numpy as np
import torch

# Check installations
try:
    from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
    print("✓ SAM available")
    SAM_AVAILABLE = True
except ImportError:
    print("⚠ SAM not installed - using fallback YOLO")
    print("For best results install: pip install git+https://github.com/facebookresearch/segment-anything.git")
    SAM_AVAILABLE = False
    from ultralytics import YOLO

class PerfectDetector:
    def __init__(self, use_sam=True):
        self.use_sam = use_sam and SAM_AVAILABLE

        if self.use_sam:
            print("Loading SAM model (best quality)...")
            # Download model if not exists
            import os
            model_path = "sam_vit_h_4b8939.pth"
            if not os.path.exists(model_path):
                print("Downloading SAM model (2.4GB, one-time only)...")
                import urllib.request
                urllib.request.urlretrieve(
                    "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth",
                    model_path
                )

            sam = sam_model_registry["vit_h"](checkpoint=model_path)
            device = "cuda" if torch.cuda.is_available() else "cpu"
            sam.to(device=device)

            self.mask_generator = SamAutomaticMaskGenerator(
                sam,
                points_per_side=32,
                pred_iou_thresh=0.86,
                stability_score_thresh=0.92,
                min_mask_region_area=500,
            )
            print(f"✓ SAM loaded on {device}")
        else:
            print("Loading YOLOv8...")
            self.model = YOLO('yolov8n.pt')
            print("✓ YOLO loaded")

    def detect(self, image):
        """Returns list of detected objects with masks and bboxes"""
        if self.use_sam:
            return self._detect_sam(image)
        else:
            return self._detect_yolo(image)

    def _detect_sam(self, image):
        """SAM detection - perfect segmentation"""
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        masks = self.mask_generator.generate(rgb)

        detections = []
        for mask_data in masks:
            mask = mask_data['segmentation']
            bbox = mask_data['bbox']  # x, y, w, h

            # Convert to x1, y1, x2, y2
            x, y, w, h = bbox
            bbox_xyxy = [x, y, x+w, y+h]

            detections.append({
                'bbox': bbox_xyxy,
                'mask': mask,
                'confidence': mask_data['predicted_iou'],
                'class': 'object',  # SAM doesn't classify, just segments
                'area': mask_data['area']
            })

        # Sort by confidence
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        return detections

    def _detect_yolo(self, image):
        """YOLO detection - faster, with classification"""
        results = self.model(image, verbose=False)

        detections = []
        for result in results:
            boxes = result.boxes

            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
            else:
                masks = [None] * len(boxes)

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                class_name = self.model.names[int(box.cls[0])]

                if confidence > 0.3:  # Lower threshold
                    detection = {
                        'bbox': [x1, y1, x2, y2],
                        'mask': masks[i] if masks[i] is not None else None,
                        'confidence': confidence,
                        'class': class_name,
                        'area': (x2-x1) * (y2-y1)
                    }
                    detections.append(detection)

        return detections


def compute_grasp(detection, image_shape):
    """
    Compute optimal grasp from detection.
    Uses actual object mask for better accuracy.
    """
    bbox = detection['bbox']
    mask = detection['mask']

    x1, y1, x2, y2 = bbox

    # If we have a mask, use it for precise grasp point
    if mask is not None:
        # Find contours from mask
        if isinstance(mask, torch.Tensor):
            mask_np = mask.cpu().numpy().astype(np.uint8) * 255
        else:
            mask_np = mask.astype(np.uint8) * 255

        # Resize mask if needed
        if mask_np.shape != image_shape[:2]:
            mask_np = cv2.resize(mask_np, (image_shape[1], image_shape[0]))

        contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Use largest contour
            contour = max(contours, key=cv2.contourArea)

            # Get oriented bounding box
            if len(contour) >= 5:
                rect = cv2.minAreaRect(contour)
                center, (width, height), angle = rect

                # Grasp along the shorter axis
                if width > height:
                    grasp_angle = angle
                else:
                    grasp_angle = angle + 90

                return {
                    'position': center,
                    'orientation': grasp_angle,
                    'width': min(width, height),
                    'confidence': detection['confidence'],
                    'type': 'mask-based'
                }

    # Fallback: use bounding box
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    width = x2 - x1
    height = y2 - y1

    # Grasp orientation based on aspect ratio
    if width > height * 1.3:
        angle = 0  # Horizontal
    elif height > width * 1.3:
        angle = 90  # Vertical
    else:
        angle = 45  # Diagonal

    return {
        'position': (cx, cy),
        'orientation': angle,
        'width': min(width, height),
        'confidence': detection['confidence'],
        'type': 'bbox-based'
    }


def main():
    # Try SAM first, fallback to YOLO
    detector = PerfectDetector(use_sam=True)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n" + "=" * 60)
    print("Perfect Object Detection + Grasp Planning")
    print("=" * 60)
    print("Press 'q' to quit")
    print("=" * 60 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect objects
        detections = detector.detect(frame)

        # Draw results
        vis = frame.copy()

        for i, det in enumerate(detections):
            # Draw mask if available
            if det['mask'] is not None:
                mask = det['mask']
                if isinstance(mask, torch.Tensor):
                    mask = mask.cpu().numpy()

                if mask.shape != frame.shape[:2]:
                    mask = cv2.resize(mask.astype(np.uint8),
                                    (frame.shape[1], frame.shape[0]))

                # Create colored overlay
                color = np.random.randint(0, 255, 3).tolist()
                mask_overlay = np.zeros_like(vis)
                mask_overlay[mask > 0] = color
                vis = cv2.addWeighted(vis, 1.0, mask_overlay, 0.3, 0)

            # Draw bounding box
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label
            label = f"{det['class']} {det['confidence']:.2f}"
            cv2.putText(vis, label, (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Compute and draw grasp
            grasp = compute_grasp(det, frame.shape)

            cx, cy = [int(v) for v in grasp['position']]

            # Grasp point
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)

            # Orientation arrow
            angle_rad = np.deg2rad(grasp['orientation'])
            length = 50
            end_x = int(cx + length * np.cos(angle_rad))
            end_y = int(cy + length * np.sin(angle_rad))
            cv2.arrowedLine(vis, (cx, cy), (end_x, end_y), (255, 0, 0), 3)

            # Gripper width
            perp_angle = angle_rad + np.pi/2
            half_width = grasp['width'] / 4
            p1 = (int(cx + half_width * np.cos(perp_angle)),
                  int(cy + half_width * np.sin(perp_angle)))
            p2 = (int(cx - half_width * np.cos(perp_angle)),
                  int(cy - half_width * np.sin(perp_angle)))
            cv2.line(vis, p1, p2, (255, 0, 0), 2)

            # Grasp info
            grasp_info = f"Grasp: {grasp['type']}"
            cv2.putText(vis, grasp_info, (x1, y2+20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        # Stats
        stats = f"Objects: {len(detections)}"
        cv2.putText(vis, stats, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Perfect Detection', vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
