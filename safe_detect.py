#!/usr/bin/env python3
"""
SAFE VERSION - No window spam, no crashes!
Simple and stable vertical crop rotation.
"""

import cv2
import numpy as np
from ultralytics import YOLO
import warnings
import os
import math

warnings.filterwarnings('ignore')
os.environ['YOLO_VERBOSE'] = 'False'

print("Loading YOLO...")
model = YOLO("yolov8n-seg.pt")
model.overrides['verbose'] = False
print("Ready! Press 'q' to quit\n")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create windows ONCE at start
MAIN_WINDOW = "Main View"
CROP_WINDOW = "Vertical Crops"
cv2.namedWindow(MAIN_WINDOW)
cv2.namedWindow(CROP_WINDOW)

ALLOWED_CLASSES = {"bottle", "wine glass", "cup"}  # limit to bottle-like objects to avoid label jumps
EXCLUDED_CLASSES = {"person", "chair", "suitcase"}  # always skip people
_prev_rotation = None  # global smoothing state
_last_roi = None  # (x1, y1, x2, y2) crop where we keep detecting
_locked_class = None  # class we stick to
_locked_miss = 0      # how many frames we missed the locked class
_candidate_class = None  # potential new class
_candidate_count = 0      # consecutive frames for candidate
CANDIDATE_REQUIRED_FRAMES = 12
CANDIDATE_MIN_CONF = 0.7


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

        # Major axis direction
        vx, vy = eigenvectors[0]
        angle_deg = math.degrees(math.atan2(vy, vx))  # from +X axis

        # Rotate so major axis becomes vertical (90 deg from +X)
        rotation = 90.0 - angle_deg

        # Normalize to [-90, 90] to reduce flips
        while rotation > 90:
            rotation -= 180
        while rotation < -90:
            rotation += 180

        # Smooth rotation to avoid jitter
        if _prev_rotation is None:
            smoothed = rotation
        else:
            alpha = 0.3  # higher -> reacts faster
            smoothed = (1 - alpha) * _prev_rotation + alpha * rotation
        _prev_rotation = smoothed

        return True, float(cx), float(cy), smoothed
    except Exception as e:
        print(f"Error in orientation: {e}")
        return False, None, None, 0.0


print("Running... Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Choose detection source: keep detection only inside last ROI; fall back to full frame if lost
    detect_frame = frame
    roi_origin = (0, 0)
    if _last_roi is not None:
        x1, y1, x2, y2 = _last_roi
        x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
        if x2i > x1i and y2i > y1i:
            detect_frame = frame[y1i:y2i, x1i:x2i]
            roi_origin = (x1i, y1i)

    try:
        results = model(detect_frame, verbose=False, conf=0.6)
    except:
        cv2.imshow(MAIN_WINDOW, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    crops = []
    chosen = None           # best area within allowed/locked classes
    fallback = None         # best area of any class (in case YOLO mislabels)
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
                # Reproject box to full-frame coordinates if detecting inside ROI
                if roi_origin != (0, 0):
                    x1 += roi_origin[0]
                    x2 += roi_origin[0]
                    y1 += roi_origin[1]
                    y2 += roi_origin[1]
                name = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                area = (x2 - x1) * (y2 - y1)

                # Always ignore excluded classes
                if name in EXCLUDED_CLASSES:
                    continue

                # Stick to locked class if set
                if _locked_class and name != _locked_class:
                    pass  # still add to candidates/fallback but lower priority

                cand = {
                    "box": (x1, y1, x2, y2),
                    "name": name,
                    "conf": conf,
                    "mask": masks.data[i].cpu().numpy() if masks is not None and len(masks.data) > i else None,
                }

                # Priority: locked_class > allowed classes
                if _locked_class and name == _locked_class and area > best_area_allowed:
                    best_area_allowed = area
                    chosen = cand
                elif not _locked_class and name in ALLOWED_CLASSES and area > best_area_allowed:
                    best_area_allowed = area
                    chosen = cand
                if area > best_area_any:
                    best_area_any = area
                    fallback = cand

            except Exception:
                continue

    target = chosen if chosen is not None else fallback

    if target:
        x1, y1, x2, y2 = target["box"]
        name = target["name"]
        conf = target["conf"]
        mask = target["mask"]

        has_angle = False
        tilt = 0.0
        if mask is not None:
            has_angle, cx, cy, tilt = compute_upright_rotation(mask, frame.shape)
        else:
            _prev_rotation = None

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"{name} {conf:.2f}", (int(x1), int(y1)-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Padding around ROI
        pad = 50
        cx1 = max(0, int(x1) - pad)
        cy1 = max(0, int(y1) - pad)
        cx2 = min(frame.shape[1], int(x2) + pad)
        cy2 = min(frame.shape[0], int(y2) + pad)
        _last_roi = (cx1, cy1, cx2, cy2)  # keep detecting inside this ROI

        crop = frame[cy1:cy2, cx1:cx2]
        if crop.size > 0 and has_angle:
            h, w = crop.shape[:2]
            if h > 20 and w > 20:
                rotation_angle = -tilt
                center = (w/2, h/2)
                M = cv2.getRotationMatrix2D(center, rotation_angle, 1.0)
                cos = abs(M[0,0])
                sin = abs(M[0,1])
                nW = int(h*sin + w*cos)
                nH = int(h*cos + w*sin)
                M[0,2] += (nW/2) - center[0]
                M[1,2] += (nH/2) - center[1]
                crop = cv2.warpAffine(crop, M, (nW, nH),
                                     borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=(40,40,40))
                print(f"[{name}] Tilt: {tilt:.1f}° → Rotated by: {rotation_angle:.1f}°")

        if crop.size > 0:
            h, w = crop.shape[:2]
            if h > 0 and w > 0:
                overlay = crop.copy()
                cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
                crop = cv2.addWeighted(overlay, 0.6, crop, 0.4, 0)
                cv2.putText(crop, f"{name.upper()}", (15, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                if has_angle:
                    cv2.putText(crop, f"Tilt: {tilt:.1f}", (15, 55),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(crop, "CORRECTED", (15, 75),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                center_x = w // 2
                cv2.line(crop, (center_x, 0), (center_x, h), (255, 255, 0), 4)
                arrow_start = 100
                arrow_end = 85
                cv2.arrowedLine(crop, (center_x, arrow_start), (center_x, arrow_end),
                               (255, 255, 0), 3, tipLength=0.5)
                crops.append(crop)
        # Update lock logic: require consecutive frames to switch class
        if _locked_class is None:
            if _candidate_class == name:
                _candidate_count += 1
            else:
                _candidate_class = name
                _candidate_count = 1
            if _candidate_count >= CANDIDATE_REQUIRED_FRAMES and conf >= CANDIDATE_MIN_CONF:
                _locked_class = name
                _locked_miss = 0
                _candidate_class = None
                _candidate_count = 0
        else:
            if name == _locked_class:
                _locked_miss = 0
                _candidate_class = None
                _candidate_count = 0
            elif conf >= CANDIDATE_MIN_CONF:
                if _candidate_class == name:
                    _candidate_count += 1
                else:
                    _candidate_class = name
                    _candidate_count = 1
                if _candidate_count >= CANDIDATE_REQUIRED_FRAMES:
                    _locked_class = name
                    _locked_miss = 0
                    _candidate_class = None
                    _candidate_count = 0
    else:
        _prev_rotation = None
        _last_roi = None
        if _locked_class:
            _locked_miss += 1
            if _locked_miss > 20:
                _locked_class = None
                _locked_miss = 0
        _candidate_class = None
        _candidate_count = 0

    # Show main frame
    cv2.imshow(MAIN_WINDOW, frame)

    # Show crops (if any)
    if crops:
        # Resize all to same height
        h_target = 300
        resized = []
        for c in crops:
            h, w = c.shape[:2]
            if h > 0:
                scale = h_target / h
                resized.append(cv2.resize(c, (int(w*scale), h_target)))

        if resized:
            combined = np.hstack(resized) if len(resized) > 1 else resized[0]
            cv2.imshow(CROP_WINDOW, combined)

    # Handle quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
print("Done!")
