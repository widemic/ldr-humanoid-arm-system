#!/usr/bin/env python3
"""
Clean Object Detection Pipeline:
1. Detect objects (any blob/contour)
2. Track them
3. Classify with YOLO on upright crop
Simple and works at any rotation!
"""

import cv2
import numpy as np
from ultralytics import YOLO

print("Loading YOLO...")
model = YOLO('yolov8n.pt')  # Regular YOLO (faster, no seg needed)
print("Ready!")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("\nSimple Detection + Tracking + Classification")
print("Press 'q' to quit\n")

# Tracker
tracker = cv2.TrackerKCF_create if hasattr(cv2, 'TrackerKCF_create') else None
tracked_objects = {}
next_id = 0

def detect_objects_simple(frame):
    """
    Simple blob detection - finds ANY object regardless of rotation.
    No YOLO needed here, just find blobs!
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Adaptive threshold
    binary = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 21, 3)

    # Clean up
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    objects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 2000 or area > 150000:  # Filter by size
            continue

        # Get bounding box
        x, y, w, h = cv2.boundingRect(contour)

        # Get rotation
        if len(contour) >= 5:
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
        else:
            angle = 0

        objects.append({
            'bbox': (x, y, w, h),
            'contour': contour,
            'angle': angle
        })

    return objects

def classify_object(frame, bbox, angle):
    """
    Classify object by cropping, rotating upright, then running YOLO.
    """
    x, y, w, h = bbox

    # Crop with margin
    margin = 20
    x1 = max(0, x - margin)
    y1 = max(0, y - margin)
    x2 = min(frame.shape[1], x + w + margin)
    y2 = min(frame.shape[0], y + h + margin)

    crop = frame[y1:y2, x1:x2]

    if crop.shape[0] < 50 or crop.shape[1] < 50:
        return "unknown", 0.0, crop

    # Rotate to upright
    if abs(angle) > 5:
        h_c, w_c = crop.shape[:2]
        center = (w_c // 2, h_c // 2)
        M = cv2.getRotationMatrix2D(center, -angle, 1.0)

        # Expand canvas to fit rotated image
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        new_w = int((h_c * sin) + (w_c * cos))
        new_h = int((h_c * cos) + (w_c * sin))

        M[0, 2] += (new_w / 2) - center[0]
        M[1, 2] += (new_h / 2) - center[1]

        crop = cv2.warpAffine(crop, M, (new_w, new_h))

    # Run YOLO on upright crop
    results = model(crop, verbose=False, conf=0.15)

    if len(results) > 0 and len(results[0].boxes) > 0:
        box = results[0].boxes[0]
        name = model.names[int(box.cls[0])]
        conf = float(box.conf[0])
        return name, conf, crop

    return "unknown", 0.0, crop

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Step 1: Detect blobs (any object, rotation-invariant)
    detected_objects = detect_objects_simple(frame)

    # Step 2: Match to existing tracked objects or create new
    new_tracked = {}

    for obj in detected_objects:
        x, y, w, h = obj['bbox']
        center = (x + w//2, y + h//2)

        # Try to match to existing tracked object
        matched = False
        for track_id, tracked in tracked_objects.items():
            tx, ty = tracked['center']
            distance = np.sqrt((center[0] - tx)**2 + (center[1] - ty)**2)

            if distance < 50:  # Within 50 pixels
                # Update existing
                new_tracked[track_id] = {
                    'bbox': obj['bbox'],
                    'center': center,
                    'angle': obj['angle'],
                    'name': tracked.get('name', 'unknown'),
                    'confidence': tracked.get('confidence', 0.0),
                    'frames': tracked.get('frames', 0) + 1
                }
                matched = True
                break

        if not matched:
            # New object - create new track
            new_tracked[next_id] = {
                'bbox': obj['bbox'],
                'center': center,
                'angle': obj['angle'],
                'name': 'unknown',
                'confidence': 0.0,
                'frames': 1
            }
            next_id += 1

    tracked_objects = new_tracked

    # Step 3: Classify tracked objects (only every 5 frames to save CPU)
    if frame_count % 5 == 0:
        for track_id, tracked in tracked_objects.items():
            if tracked['frames'] > 3:  # Only classify stable objects
                name, conf, crop = classify_object(
                    frame, tracked['bbox'], tracked['angle'])

                tracked['name'] = name
                tracked['confidence'] = conf

    # Draw
    for track_id, tracked in tracked_objects.items():
        x, y, w, h = tracked['bbox']
        name = tracked['name']
        conf = tracked['confidence']
        angle = tracked['angle']

        # Color by confidence
        if conf > 0.7:
            color = (0, 255, 0)  # Green - high
        elif conf > 0.4:
            color = (0, 255, 255)  # Yellow - medium
        elif conf > 0.0:
            color = (0, 165, 255)  # Orange - low
        else:
            color = (128, 128, 128)  # Gray - not classified yet

        # Draw box
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

        # Label
        label = f"ID:{track_id} {name} {conf:.2f} @{angle:.0f}°"
        cv2.putText(frame, label, (x, y-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw center
        cv2.circle(frame, tracked['center'], 5, (0, 0, 255), -1)

        # Draw rotation arrow
        cx, cy = tracked['center']
        length = 40
        angle_rad = np.deg2rad(angle)
        end_x = int(cx + length * np.cos(angle_rad))
        end_y = int(cy + length * np.sin(angle_rad))
        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (255, 0, 0), 2)

    # Stats
    cv2.putText(frame, f"Tracking: {len(tracked_objects)}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow('Simple Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
