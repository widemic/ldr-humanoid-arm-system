#!/usr/bin/env python3
"""
Object detection with Test-Time Augmentation (TTA)
Detects objects at multiple rotations to handle sideways objects better
"""

import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLO
print("Loading YOLO...")
model = YOLO('yolov8n-seg.pt')
print("Ready!")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("\nPress 'q' to quit")
print("Press 't' to toggle TTA (Test-Time Augmentation)\n")

use_tta = False

def rotate_image(image, angle):
    """Rotate image by angle degrees"""
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated, M

def rotate_bbox(bbox, M, img_shape):
    """Rotate bounding box coordinates"""
    x1, y1, x2, y2 = bbox

    # Get all 4 corners
    corners = np.array([
        [x1, y1],
        [x2, y1],
        [x2, y2],
        [x1, y2]
    ])

    # Add homogeneous coordinate
    ones = np.ones(shape=(len(corners), 1))
    corners_hom = np.hstack([corners, ones])

    # Transform
    transformed = M.dot(corners_hom.T).T

    # Get new bbox
    x_coords = transformed[:, 0]
    y_coords = transformed[:, 1]

    new_x1 = max(0, int(np.min(x_coords)))
    new_y1 = max(0, int(np.min(y_coords)))
    new_x2 = min(img_shape[1], int(np.max(x_coords)))
    new_y2 = min(img_shape[0], int(np.max(y_coords)))

    return [new_x1, new_y1, new_x2, new_y2]

while True:
    ret, frame = cap.read()
    if not ret:
        break

    all_detections = []

    if use_tta:
        # Try multiple rotations
        angles = [0, 90, 180, 270]

        for angle in angles:
            rotated, M = rotate_image(frame, angle)
            results = model(rotated, verbose=False, conf=0.15, iou=0.5)

            for result in results:
                boxes = result.boxes
                masks = result.masks

                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = model.names[cls]

                    if name in ['person', 'keyboard', 'laptop', 'dining table']:
                        continue

                    # Rotate bbox back to original orientation
                    M_inv = cv2.getRotationMatrix2D(
                        (frame.shape[1]//2, frame.shape[0]//2), -angle, 1.0
                    )
                    bbox_orig = rotate_bbox([x1, y1, x2, y2], M_inv, frame.shape)

                    all_detections.append({
                        'bbox': bbox_orig,
                        'class': name,
                        'confidence': conf,
                        'rotation': angle,
                        'mask': masks.data[i] if masks is not None else None
                    })

        # Remove duplicates (keep highest confidence)
        unique_detections = []
        for det in sorted(all_detections, key=lambda x: x['confidence'], reverse=True):
            is_duplicate = False
            for unique in unique_detections:
                # Check if bboxes overlap significantly
                x1a, y1a, x2a, y2a = det['bbox']
                x1b, y1b, x2b, y2b = unique['bbox']

                # Calculate IoU
                xi1 = max(x1a, x1b)
                yi1 = max(y1a, y1b)
                xi2 = min(x2a, x2b)
                yi2 = min(y2a, y2b)

                if xi2 > xi1 and yi2 > yi1:
                    inter_area = (xi2 - xi1) * (yi2 - yi1)
                    box1_area = (x2a - x1a) * (y2a - y1a)
                    box2_area = (x2b - x1b) * (y2b - y1b)
                    union_area = box1_area + box2_area - inter_area
                    iou = inter_area / union_area if union_area > 0 else 0

                    if iou > 0.5:
                        is_duplicate = True
                        break

            if not is_duplicate:
                unique_detections.append(det)

        detections = unique_detections

    else:
        # Normal detection (no TTA)
        results = model.track(frame, verbose=False, conf=0.15, iou=0.5, persist=True)

        detections = []
        for result in results:
            boxes = result.boxes
            masks = result.masks

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls]
                track_id = int(box.id[0]) if box.id is not None else -1

                if name in ['person', 'keyboard', 'laptop', 'dining table']:
                    continue

                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'class': name,
                    'confidence': conf,
                    'track_id': track_id,
                    'rotation': 0,
                    'mask': masks.data[i] if masks is not None else None
                })

    # Draw detections
    vis = frame.copy()

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det['bbox']]
        name = det['class']
        conf = det['confidence']

        # Color based on confidence
        if conf > 0.7:
            color = (0, 255, 0)  # Green - high confidence
        elif conf > 0.4:
            color = (0, 255, 255)  # Yellow - medium
        else:
            color = (0, 165, 255)  # Orange - low (likely rotated)

        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

        # Label with rotation info if TTA
        if use_tta and det['rotation'] > 0:
            label = f"{name} {conf:.2f} (rot:{det['rotation']}°)"
        elif not use_tta and 'track_id' in det:
            label = f"{name} {conf:.2f} ID:{det['track_id']}"
        else:
            label = f"{name} {conf:.2f}"

        cv2.putText(vis, label, (x1, y1-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Compute rotation if mask available
        if det['mask'] is not None:
            mask = det['mask'].cpu().numpy() if hasattr(det['mask'], 'cpu') else det['mask']
            mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
            mask_uint8 = (mask_resized * 255).astype(np.uint8)

            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                cnt = max(contours, key=cv2.contourArea)

                if len(cnt) >= 5:
                    rect = cv2.minAreaRect(cnt)
                    box_points = cv2.boxPoints(rect)
                    box_points = np.int32(box_points)

                    cv2.drawContours(vis, [box_points], 0, (255, 0, 0), 2)

                    (cx, cy), (w, h), angle = rect

                    # Draw orientation
                    angle_rad = np.deg2rad(angle)
                    length = 50
                    end_x = int(cx + length * np.cos(angle_rad))
                    end_y = int(cy + length * np.sin(angle_rad))
                    cv2.arrowedLine(vis, (int(cx), int(cy)), (end_x, end_y),
                                   (255, 0, 0), 3)

                    cv2.putText(vis, f"angle: {angle:.1f}°", (int(cx)+10, int(cy)+10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Status
    mode_text = "TTA: ON (better for rotated)" if use_tta else "TTA: OFF (faster)"
    cv2.putText(vis, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(vis, f"Detected: {len(detections)}", (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Legend
    cv2.putText(vis, "Green=High conf, Yellow=Med, Orange=Low (rotated?)", (10, vis.shape[0]-10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    cv2.imshow('Object Detection', vis)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('t'):
        use_tta = not use_tta
        print(f"TTA: {'ON' if use_tta else 'OFF'}")

cap.release()
cv2.destroyAllWindows()
