#!/usr/bin/env python3
"""
SINGLE OBJECT DETECTION - Focus on ONE object at a time!

Features:
- Processes only the LARGEST or CLOSEST object
- Frame skip for stability
- Perfect vertical crop rotation
"""

import cv2
import numpy as np
from ultralytics import YOLO
import warnings
import os
import time
import math

warnings.filterwarnings('ignore')
os.environ['YOLO_VERBOSE'] = 'False'

print("Loading YOLO...")
model = YOLO("yolov8n-seg.pt")
model.overrides['verbose'] = False
print("Ready!\n")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Windows
MAIN_WINDOW = "Main View - Single Object"
CROP_WINDOW = "Vertical Crop - ONE Object"
cv2.namedWindow(MAIN_WINDOW)
cv2.namedWindow(CROP_WINDOW)

# Processing control
FRAME_SKIP = 2  # Process every N frames
frame_count = 0
last_process_time = 0
PROCESS_DELAY = 0.05  # 50ms delay between processing
ALLOWED_CLASSES = {"bottle", "wine glass", "cup"}  # focus only on bottle-like objects
EXCLUDED_CLASSES = {"person"}  # always skip people
_last_roi = None  # (x1, y1, x2, y2) where we keep detecting
_prev_rotation = None


def compute_upright_rotation(mask, frame_shape):
    """
    Compute rotation to make the major axis vertical using PCA (more stable than minAreaRect).
    Returns (ok, cx, cy, rotation_deg_smoothed).
    """
    global _prev_rotation
    try:
        mask_resized = cv2.resize(mask, (frame_shape[1], frame_shape[0]))
        mask_uint8 = (mask_resized * 255).astype(np.uint8)

        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, 0.0

        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 5:
            return False, None, None, 0.0

        pts = contour.reshape(-1, 2).astype(np.float32)
        mean, eigenvectors, _ = cv2.PCACompute2(pts, mean=None)
        cx, cy = mean[0]

        vx, vy = eigenvectors[0]
        angle_deg = math.degrees(math.atan2(vy, vx))
        rotation = 90.0 - angle_deg  # align major axis to vertical

        while rotation > 90:
            rotation -= 180
        while rotation < -90:
            rotation += 180

        # Smooth rotation to avoid jitter
        if _prev_rotation is None:
            smoothed = rotation
        else:
            alpha = 0.3
            smoothed = (1 - alpha) * _prev_rotation + alpha * rotation
        _prev_rotation = smoothed

        return True, float(cx), float(cy), smoothed
    except Exception as e:
        print(f"Error: {e}")
        return False, None, None, 0.0


print("=" * 70)
print("  SINGLE OBJECT MODE - Focus on ONE object!")
print("=" * 70)
print("\nTips:")
print("  - Hold ONE object clearly visible")
print("  - Keep it centered in frame")
print("  - System focuses on LARGEST object only")
print("\nPress 'q' to quit\n")
print("=" * 70 + "\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Frame skip for stability
    if frame_count % FRAME_SKIP != 0:
        cv2.imshow(MAIN_WINDOW, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # Delay for stability
    current_time = time.time()
    if current_time - last_process_time < PROCESS_DELAY:
        cv2.imshow(MAIN_WINDOW, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    last_process_time = current_time

    # Run detection (prefer detecting only inside last ROI)
    detect_frame = frame
    roi_origin = (0, 0)
    if _last_roi is not None:
        x1r, y1r, x2r, y2r = map(int, _last_roi)
        if x2r > x1r and y2r > y1r:
            detect_frame = frame[y1r:y2r, x1r:x2r]
            roi_origin = (x1r, y1r)

    try:
        results = model(detect_frame, verbose=False, conf=0.25)
    except:
        cv2.imshow(MAIN_WINDOW, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # Find the BEST object (prefer allowed classes; fallback to largest overall)
    best_detection = None
    best_allowed = None
    best_area_allowed = -1
    best_area_any = -1

    for result in results:
        if not hasattr(result, 'boxes') or result.boxes is None:
            continue

        boxes = result.boxes
        masks = result.masks

        for i, box in enumerate(boxes):
            try:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                if roi_origin != (0, 0):
                    x1 += roi_origin[0]
                    x2 += roi_origin[0]
                    y1 += roi_origin[1]
                    y2 += roi_origin[1]
                name = model.names[int(box.cls[0])]
                conf = float(box.conf[0])

                # Always ignore excluded classes
                if name in EXCLUDED_CLASSES:
                    continue

                # Calculate area
                area = (x2 - x1) * (y2 - y1)

                cand = {
                    'box': (x1, y1, x2, y2),
                    'name': name,
                    'conf': conf,
                    'mask': masks.data[i].cpu().numpy() if masks is not None and len(masks.data) > i else None
                }

                if name in ALLOWED_CLASSES and area > best_area_allowed:
                    best_area_allowed = area
                    best_allowed = cand
                if area > best_area_any:
                    best_area_any = area
                    best_detection = cand

            except:
                continue

    # Process ONLY the best detection (prefer allowed, else fallback)
    active = best_allowed if best_allowed is not None else best_detection
    if active is not None:
        x1, y1, x2, y2 = active['box']
        name = active['name']
        conf = active['conf']
        mask = active['mask']
        _last_roi = (max(0, int(x1) - 50),
                     max(0, int(y1) - 50),
                     min(frame.shape[1], int(x2) + 50),
                     min(frame.shape[0], int(y2) + 50))

        # Get orientation
        has_angle = False
        tilt = 0.0
        cx = cy = None

        if mask is not None:
            has_angle, cx, cy, tilt = compute_upright_rotation(mask, frame.shape)
        else:
            _prev_rotation = None

        # Draw on main frame
        color = (0, 255, 0) if conf > 0.5 else (0, 255, 255)
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)

        label = f"{name} {conf:.2f}"
        cv2.putText(frame, label, (int(x1), int(y1) - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        if has_angle and cx is not None:
            # Draw center point
            cv2.circle(frame, (int(cx), int(cy)), 10, (0, 0, 255), -1)

            # Draw orientation arrow
            angle_rad = np.radians(90 + tilt)
            length = 80
            ex = int(cx + length * np.cos(angle_rad))
            ey = int(cy + length * np.sin(angle_rad))

            cv2.arrowedLine(frame, (int(cx), int(cy)), (ex, ey),
                          (255, 0, 255), 4, tipLength=0.3)

            # Show tilt angle
            cv2.putText(frame, f"Tilt: {tilt:.1f}°",
                       (int(cx) + 25, int(cy) - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)

        # Extract and rotate crop
        pad = 80
        crop_x1 = max(0, int(x1) - pad)
        crop_y1 = max(0, int(y1) - pad)
        crop_x2 = min(frame.shape[1], int(x2) + pad)
        crop_y2 = min(frame.shape[0], int(y2) + pad)

        crop = frame[crop_y1:crop_y2, crop_x1:crop_x2].copy()

        if crop.size > 0 and has_angle:
            h, w = crop.shape[:2]

            if h > 30 and w > 30:
                # Rotate opposite to measured tilt to make it upright
                rotation_angle = -tilt

                print(f"\n[{name}] Tilt: {tilt:.1f}° → Rotating by: {rotation_angle:.1f}°")

                center = (w / 2.0, h / 2.0)
                M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)

                # New canvas size
                cos = abs(M[0, 0])
                sin = abs(M[0, 1])
                nW = int(h * sin + w * cos)
                nH = int(h * cos + w * sin)

                # Adjust translation
                M[0, 2] += (nW / 2.0) - center[0]
                M[1, 2] += (nH / 2.0) - center[1]

                # Apply rotation
                crop = cv2.warpAffine(crop, M, (nW, nH),
                                     borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=(50, 50, 50))

        # Add visual guides to crop
        if crop.size > 0:
            h, w = crop.shape[:2]

            if h > 0 and w > 0:
                # Dark overlay at top
                overlay = crop.copy()
                text_height = 100
                cv2.rectangle(overlay, (0, 0), (w, text_height), (0, 0, 0), -1)
                crop = cv2.addWeighted(overlay, 0.7, crop, 0.3, 0)

                # Title
                cv2.putText(crop, name.upper(), (20, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

                if has_angle:
                    cv2.putText(crop, f"Tilt: {tilt:.1f}°", (20, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv2.putText(crop, f"Corrected: {-tilt:.1f}°", (20, 95),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # THICK vertical reference line (CYAN)
                center_x = w // 2
                cv2.line(crop, (center_x, 0), (center_x, h), (255, 255, 0), 5)

                # Horizontal reference at center
                center_y = h // 2
                cv2.line(crop, (0, center_y), (w, center_y), (100, 100, 100), 1)

                # UP arrow
                arrow_y_start = 130
                arrow_y_end = 110
                cv2.arrowedLine(crop, (center_x, arrow_y_start), (center_x, arrow_y_end),
                               (255, 255, 0), 5, tipLength=0.6)

                cv2.putText(crop, "UP", (center_x - 30, arrow_y_end - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                # Resize for display
                max_display_height = 500
                if h > max_display_height:
                    scale = max_display_height / h
                    new_w = int(w * scale)
                    crop = cv2.resize(crop, (new_w, max_display_height))

                cv2.imshow(CROP_WINDOW, crop)

    else:
        _last_roi = None

    # Status text
    status = "Detected: 1 object" if active is not None else "No object detected"
    status_color = (0, 255, 0) if active is not None else (0, 0, 255)

    cv2.putText(frame, status, (20, 40),
               cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

    cv2.putText(frame, "Focus on ONE object", (20, 80),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow(MAIN_WINDOW, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\nDone!")
