#!/usr/bin/env python3
"""
Simple YOLO-based object detection + grasp position
Much more reliable than contour detection!
"""

import cv2
import numpy as np

# Check if ultralytics is installed
try:
    from ultralytics import YOLO
    print("✓ YOLO available")
except ImportError:
    print("ERROR: YOLO not installed!")
    print("Install with: pip install ultralytics")
    exit(1)

print("Loading YOLO model...")
model = YOLO('yolov8n.pt')  # Nano model - fast and small
print("✓ Model loaded")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("\n" + "=" * 60)
print("YOLO Object Detection + Grasp Planning")
print("=" * 60)
print("Press 'q' to quit")
print("=" * 60 + "\n")

def compute_grasp_for_bbox(bbox, class_name):
    """
    Compute grasp position and orientation from bounding box.
    Simple heuristic based on object type.
    """
    x1, y1, x2, y2 = bbox

    # Center of bounding box
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    # Size
    width = x2 - x1
    height = y2 - y1

    # Orientation (0 = horizontal, 90 = vertical)
    if width > height * 1.5:
        # Wide object (bottle lying down, etc.)
        orientation = 0  # Horizontal grasp
        grasp_type = "horizontal"
    elif height > width * 1.5:
        # Tall object (bottle standing, etc.)
        orientation = 90  # Vertical grasp
        grasp_type = "vertical"
    else:
        # Square-ish object
        orientation = 45  # Diagonal
        grasp_type = "top-down"

    # Object-specific adjustments
    if class_name in ['bottle', 'cup', 'wine glass']:
        # Cylindrical objects - grasp around the middle
        grasp_type = "cylindrical"
        orientation = 0
    elif class_name in ['book', 'laptop', 'cell phone']:
        # Flat objects - pinch grasp from edge
        grasp_type = "edge"
        orientation = 0 if width > height else 90
    elif class_name in ['apple', 'orange', 'sports ball']:
        # Round objects - top-down grasp
        grasp_type = "top-down"
        orientation = 0

    return {
        'position': (cx, cy),
        'orientation': orientation,
        'type': grasp_type,
        'width': min(width, height),
        'height': max(width, height)
    }

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = model(frame, verbose=False)

    # Process detections
    detections = []
    for result in results:
        boxes = result.boxes

        for box in boxes:
            # Get box coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # Get class and confidence
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = model.names[class_id]

            # Only keep detections with >50% confidence
            if confidence > 0.5:
                detections.append({
                    'bbox': (x1, y1, x2, y2),
                    'class': class_name,
                    'confidence': confidence
                })

    # Draw detections and grasps
    vis = frame.copy()

    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        class_name = det['class']
        confidence = det['confidence']

        # Draw bounding box
        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        # Compute grasp
        grasp = compute_grasp_for_bbox(det['bbox'], class_name)

        # Draw grasp point
        cx, cy = grasp['position']
        cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)

        # Draw orientation arrow
        angle_rad = np.deg2rad(grasp['orientation'])
        arrow_length = 50
        end_x = int(cx + arrow_length * np.cos(angle_rad))
        end_y = int(cy + arrow_length * np.sin(angle_rad))
        cv2.arrowedLine(vis, (cx, cy), (end_x, end_y), (255, 0, 0), 3)

        # Draw grasp width indicator
        perp_angle = angle_rad + np.pi/2
        half_width = grasp['width'] / 4
        p1_x = int(cx + half_width * np.cos(perp_angle))
        p1_y = int(cy + half_width * np.sin(perp_angle))
        p2_x = int(cx - half_width * np.cos(perp_angle))
        p2_y = int(cy - half_width * np.sin(perp_angle))
        cv2.line(vis, (p1_x, p1_y), (p2_x, p2_y), (255, 0, 0), 2)

        # Label
        label = f"{class_name} {confidence:.2f}"
        cv2.putText(vis, label, (int(x1), int(y1) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Grasp info
        grasp_text = f"Grasp: {grasp['type']}"
        cv2.putText(vis, grasp_text, (int(x1), int(y2) + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Stats
    stats = f"Objects detected: {len(detections)}"
    cv2.putText(vis, stats, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow('YOLO Grasp Detection', vis)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Done!")
