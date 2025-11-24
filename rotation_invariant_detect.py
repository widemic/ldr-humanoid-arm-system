#!/usr/bin/env python3
"""
Rotation-Invariant Object Detection
Uses feature matching instead of YOLO's rotation-biased detection
Better for manipulation tasks where objects can be in any orientation
"""

import cv2
import numpy as np
from ultralytics import YOLO

print("Loading YOLO...")
model = YOLO('yolov8n-seg.pt')
print("Ready!")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Store reference images when user presses 'c' to capture
reference_objects = {}
next_ref_id = 0

# ORB feature detector (rotation invariant)
orb = cv2.ORB_create(nfeatures=500)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

print("\nControls:")
print("  'c' - Capture current object as reference (for rotation-invariant tracking)")
print("  'r' - Reset all references")
print("  'q' - Quit\n")

def extract_object_features(mask, image):
    """Extract ORB features from object region"""
    # Create ROI from mask
    mask_uint8 = (mask * 255).astype(np.uint8)

    # Find keypoints and descriptors
    keypoints, descriptors = orb.detectAndCompute(image, mask=mask_uint8)

    return keypoints, descriptors, mask_uint8

def match_to_references(keypoints, descriptors):
    """Match current object to reference objects"""
    if len(reference_objects) == 0:
        return None, 0

    best_match_id = None
    best_match_score = 0

    for ref_id, ref_data in reference_objects.items():
        if ref_data['descriptors'] is None or descriptors is None:
            continue

        # Match features
        matches = bf.match(ref_data['descriptors'], descriptors)

        if len(matches) > 10:  # Need at least 10 matches
            # Score based on number of good matches
            score = len(matches)

            if score > best_match_score:
                best_match_score = score
                best_match_id = ref_id

    return best_match_id, best_match_score

# Track objects across frames
tracked_objects = {}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO (just for initial segmentation)
    results = model(frame, verbose=False, conf=0.10)

    current_detections = []

    for result in results:
        boxes = result.boxes
        masks = result.masks

        if masks is None:
            continue

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls]

            # Filter
            if name in ['person', 'keyboard', 'laptop', 'dining table']:
                continue

            # Get mask
            mask = masks.data[i].cpu().numpy()
            mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))

            # Extract features
            keypoints, descriptors, mask_uint8 = extract_object_features(mask_resized, frame)

            # Try to match to references (rotation-invariant!)
            ref_match_id, match_score = match_to_references(keypoints, descriptors)

            if ref_match_id is not None:
                # Matched to a reference - use reference name regardless of YOLO's guess
                ref_name = reference_objects[ref_match_id]['name']
                label = f"{ref_name} (ref:{ref_match_id}) matches:{match_score}"
                box_color = (0, 255, 0)  # Green - matched to reference
                actual_name = ref_name
            else:
                # No reference match - use YOLO's detection
                label = f"{name} {conf:.2f}"
                if conf < 0.3:
                    box_color = (0, 165, 255)  # Orange - low conf
                elif conf < 0.5:
                    box_color = (0, 255, 255)  # Yellow
                else:
                    box_color = (0, 255, 0)  # Green
                actual_name = name

            # Draw
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), box_color, 2)
            cv2.putText(frame, label, (int(x1), int(y1)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            # Get rotation from mask
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                cnt = max(contours, key=cv2.contourArea)

                if len(cnt) >= 5:
                    rect = cv2.minAreaRect(cnt)
                    box_points = cv2.boxPoints(rect)
                    box_points = np.int32(box_points)

                    # Draw rotated box
                    cv2.drawContours(frame, [box_points], 0, (255, 0, 0), 2)

                    (cx, cy), (w, h), angle = rect

                    # Draw center
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                    # Draw orientation arrow
                    angle_rad = np.deg2rad(angle)
                    length = 60
                    end_x = int(cx + length * np.cos(angle_rad))
                    end_y = int(cy + length * np.sin(angle_rad))
                    cv2.arrowedLine(frame, (int(cx), int(cy)), (end_x, end_y),
                                   (255, 0, 0), 3)

                    cv2.putText(frame, f"angle: {angle:.1f}°", (int(cx)+10, int(cy)+10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Store for capture
                    current_detections.append({
                        'name': actual_name,
                        'bbox': [x1, y1, x2, y2],
                        'keypoints': keypoints,
                        'descriptors': descriptors,
                        'mask': mask_uint8,
                        'angle': angle,
                        'center': (cx, cy)
                    })

    # Show stats
    cv2.putText(frame, f"Objects: {len(current_detections)} | References: {len(reference_objects)}",
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Instructions
    cv2.putText(frame, "Press 'c' to capture reference, 'r' to reset", (10, frame.shape[0]-10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow('Rotation-Invariant Detection', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        # Capture first detected object as reference
        if len(current_detections) > 0:
            obj = current_detections[0]
            reference_objects[next_ref_id] = {
                'name': obj['name'],
                'descriptors': obj['descriptors'],
                'keypoints': obj['keypoints']
            }
            print(f"Captured reference {next_ref_id}: {obj['name']}")
            next_ref_id += 1
    elif key == ord('r'):
        reference_objects = {}
        next_ref_id = 0
        print("References cleared")

cap.release()
cv2.destroyAllWindows()
