#!/usr/bin/env python3
"""
YOLOv8 segmentation + tracking
Rotate crops so that each object's long axis stays upright (vertical) in the crop.

- Orientation is computed once per frame from the ORIGINAL frame mask (never from the rotated crop),
  so we don't get the "double rotation" feedback effect.
- For each object we take a crop from the raw frame, then rotate that crop by -tilt to cancel
  whatever tilt the bottle currently has in camera space.
"""

import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLO
print("Loading YOLO...")
model = YOLO("yolov8n-seg.pt")  # segmentation model for better masks
print("Ready!")

# Tracking state
tracked_objects = {}

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Controls:")
print("  'q' - Quit")
print("  'd' - Toggle debug views")
print("  'c' - Toggle crop view")

show_debug = False
show_crops = False


def get_long_axis_angle(mask: np.ndarray, frame_shape):
    """Return (has_orientation, center_x, center_y, long_axis_angle_deg).

    long_axis_angle_deg: orientation of the LONG side of the min-area-rect, in [0, 180) degrees from x-axis.
    This is measured in the ORIGINAL frame coordinate system.
    """
    # Resize mask to frame size and binarize
    mask_resized = cv2.resize(mask, (frame_shape[1], frame_shape[0]))
    mask_uint8 = (mask_resized * 255).astype(np.uint8)

    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None, None, 0.0

    cnt = max(contours, key=cv2.contourArea)
    if len(cnt) < 5:
        return False, None, None, 0.0

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w_rect, h_rect), _ = rect

    # Get the 4 rectangle corners
    box_pts = cv2.boxPoints(rect)
    box_pts = np.array(box_pts, dtype=np.float32)

    # Edges p0->p1 and p1->p2
    v0 = box_pts[1] - box_pts[0]
    v1 = box_pts[2] - box_pts[1]

    len0 = np.linalg.norm(v0)
    len1 = np.linalg.norm(v1)

    # Long side vector
    long_vec = v0 if len0 >= len1 else v1

    # Angle of long side in degrees from x-axis
    long_axis_angle = np.degrees(np.arctan2(long_vec[1], long_vec[0]))

    # Normalize to [0, 180)
    if long_axis_angle < 0:
        long_axis_angle += 180.0

    return True, cx, cy, long_axis_angle


while True:
    ret, frame = cap.read()
    if not ret:
        break

    debug_crops = []

    results = model.track(frame, verbose=False, conf=0.10, iou=0.5, persist=True)

    detection_count = 0

    for result in results:
        boxes = result.boxes
        masks = result.masks

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls]

            track_id = int(box.id[0]) if box.id is not None else -1

            # Filter classes you don't care about
            if name in ["person", "keyboard", "laptop", "dining table"]:
                continue

            detection_count += 1

            if track_id not in tracked_objects:
                tracked_objects[track_id] = {
                    "initial_class": name,
                    "initial_conf": conf,
                    "frames_tracked": 0,
                    "low_conf_warnings": 0,
                }

            tracked_objects[track_id]["frames_tracked"] += 1
            tracked_objects[track_id]["last_conf"] = conf
            tracked_objects[track_id]["current_class"] = name

            warning = ""
            box_color = (0, 255, 0)

            if conf < 0.3:
                warning = " [LOW CONF!]"
                box_color = (0, 165, 255)
                tracked_objects[track_id]["low_conf_warnings"] += 1
            elif conf < 0.5:
                warning = " [uncertain]"
                box_color = (0, 255, 255)

            # --- orientation from mask in ORIGINAL frame ---
            has_orientation = False
            cx = cy = None
            long_axis_angle = 0.0
            rotation_angle = 0.0
            display_tilt = 0.0

            if masks is not None:
                mask = masks.data[i].cpu().numpy()
                has_orientation, cx, cy, long_axis_angle = get_long_axis_angle(mask, frame.shape)

                if has_orientation:
                    # For a perfectly upright bottle, long_axis_angle ≈ 90°.
                    # Tilt-from-vertical is how far we are from 90°.
                    display_tilt = long_axis_angle - 90.0  # negative = leans right, positive = leans left

                    # To make the bottle upright in the crop, we rotate the crop by -display_tilt.
                    rotation_angle = -display_tilt

                    # Optional: clamp crazy values when mask is noisy
                    if rotation_angle > 89.0:
                        rotation_angle = 89.0
                    if rotation_angle < -89.0:
                        rotation_angle = -89.0

            # --- crop from RAW frame (never a previously rotated crop) ---
            crop_x1 = max(0, int(x1) - 20)
            crop_y1 = max(0, int(y1) - 20)
            crop_x2 = min(frame.shape[1], int(x2) + 20)
            crop_y2 = min(frame.shape[0], int(y2) + 20)

            crop = frame[crop_y1:crop_y2, crop_x1:crop_x2]
            crop_upright = crop

            if has_orientation and crop.size > 0 and abs(rotation_angle) > 2:
                h_c, w_c = crop.shape[:2]
                center_c = (w_c // 2, h_c // 2)

                M = cv2.getRotationMatrix2D(center_c, rotation_angle, 1.0)

                cos = np.abs(M[0, 0])
                sin = np.abs(M[0, 1])
                new_w = int((h_c * sin) + (w_c * cos))
                new_h = int((h_c * cos) + (w_c * sin))

                M[0, 2] += (new_w / 2) - center_c[0]
                M[1, 2] += (new_h / 2) - center_c[1]

                crop_upright = cv2.warpAffine(crop, M, (new_w, new_h))

            # Debug crops
            if show_crops and crop_upright is not None and crop_upright.size > 0:
                crop_dbg = crop_upright.copy()
                if has_orientation:
                    txt = f"ID:{track_id} tilt:{display_tilt:.1f}° rot:{rotation_angle:.1f}°"
                else:
                    txt = f"ID:{track_id} (no angle)"
                cv2.putText(crop_dbg, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                debug_crops.append(crop_dbg)

            # If class changed, re-check on upright crop
            if (
                name != tracked_objects[track_id]["initial_class"]
                and crop_upright is not None
                and crop_upright.shape[0] > 50
                and crop_upright.shape[1] > 50
            ):
                crop_results = model(crop_upright, verbose=False, conf=0.05)
                if len(crop_results) > 0 and len(crop_results[0].boxes) > 0:
                    crop_box = crop_results[0].boxes[0]
                    crop_name = model.names[int(crop_box.cls[0])]
                    crop_conf = float(crop_box.conf[0])

                    if crop_name == tracked_objects[track_id]["initial_class"]:
                        name = crop_name
                        conf = crop_conf
                        warning += f" [crop-verified: {crop_name}]"
                        box_color = (255, 0, 255)
                    else:
                        warning += (
                            f" (was {tracked_objects[track_id]['initial_class']}, "
                            f"crop says: {crop_name})"
                        )
                        box_color = (0, 0, 255)
                else:
                    warning += f" (was {tracked_objects[track_id]['initial_class']})"
                    box_color = (0, 0, 255)

            # Draw main box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), box_color, 2)
            label = f"{name} {conf:.2f} ID:{track_id}{warning}"
            cv2.putText(
                frame,
                label,
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                box_color,
                2,
            )

            # Draw orientation overlay in ORIGINAL frame
            if has_orientation:
                # Recompute rect from mask for drawing only
                mask = masks.data[i].cpu().numpy()
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                mask_uint8 = (mask_resized * 255).astype(np.uint8)
                contours, _ = cv2.findContours(
                    mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                if contours:
                    cnt = max(contours, key=cv2.contourArea)
                    if len(cnt) >= 5:
                        rect_draw = cv2.minAreaRect(cnt)
                        box_for_draw = cv2.boxPoints(rect_draw)
                        box_for_draw = np.int32(box_for_draw)
                        cv2.drawContours(frame, [box_for_draw], 0, (255, 0, 0), 2)

                if cx is not None and cy is not None:
                    cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)

                    ang_rad = np.radians(long_axis_angle)
                    L = 60
                    ex = int(cx + L * np.cos(ang_rad))
                    ey = int(cy + L * np.sin(ang_rad))
                    cv2.arrowedLine(
                        frame, (int(cx), int(cy)), (ex, ey), (255, 0, 0), 2
                    )

                    cv2.putText(
                        frame,
                        f"axis:{long_axis_angle:.1f}° tilt:{display_tilt:.1f}°",
                        (int(cx) + 10, int(cy) + 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 0, 0),
                        1,
                    )

    # HUD / debug text
    cv2.putText(
        frame,
        f"Detected: {detection_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    if show_debug:
        y = 60
        cv2.putText(
            frame,
            f"Debug: ON | Crops: {show_crops}",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
        y += 20
        for tid, info in tracked_objects.items():
            txt = (
                f"ID:{tid} {info['initial_class']} "
                f"frames:{info['frames_tracked']} "
                f"warnings:{info['low_conf_warnings']}"
            )
            cv2.putText(
                frame,
                txt,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
                1,
            )
            y += 18

    cv2.imshow("Object Detection + Upright Crops", frame)

    if show_crops and debug_crops:
        crop_height = 150
        row = []
        for c in debug_crops:
            h, w = c.shape[:2]
            if h <= 0:
                continue
            scale = crop_height / h
            new_w = int(w * scale)
            row.append(cv2.resize(c, (new_w, crop_height)))
        if row:
            crops_display = np.hstack(row) if len(row) > 1 else row[0]
            cv2.imshow("Debug: Crops", crops_display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("d"):
        show_debug = not show_debug
        print(f"Debug view: {'ON' if show_debug else 'OFF'}")
    elif key == ord("c"):
        show_crops = not show_crops
        print(f"Crop view: {'ON' if show_crops else 'OFF'}")
        if not show_crops:
            cv2.destroyWindow("Debug: Crops")

cap.release()
cv2.destroyAllWindows()
