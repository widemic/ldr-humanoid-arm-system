#!/usr/bin/env python3
"""
simple2_detect: Adds a more robust orientation estimator that blends
min-area-rect, PCA and ellipse fits, then keeps a separate upright crop
for reclassification while still displaying the real object angle.
"""

import cv2
import numpy as np
from ultralytics import YOLO

print("Loading YOLO...")
model = YOLO("yolov8n-seg.pt")
print("Ready!")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("\nControls:")
print("  'q' - Quit")
print("  'd' - Toggle debug info")
print("  'c' - Toggle crop panel (raw + upright)\n")

tracked_objects = {}
show_debug = False
show_crops = False


def angle_average_deg(angles):
    """Average 'axial' angles (0° == 180°) in degrees."""
    if not angles:
        return None

    doubled = np.deg2rad(np.array(angles) * 2.0)
    sin_sum = np.sin(doubled).sum()
    cos_sum = np.cos(doubled).sum()

    if np.isclose(sin_sum, 0.0) and np.isclose(cos_sum, 0.0):
        return angles[0]

    avg = 0.5 * np.rad2deg(np.arctan2(sin_sum, cos_sum))
    if avg < 0:
        avg += 180.0
    return avg


def compute_orientation(mask, frame_shape):
    """
    Returns a dict with:
        has: bool
        center: (x, y)
        angle: orientation of the long axis [0, 180)
        rotation: CCW rotation needed to make long axis vertical
        tilt: signed tilt-from-vertical [-90, 90)
        box_points: 4x2 array for drawing
        sources: dict of intermediate angles (min_rect / pca / ellipse)
    """
    mask_resized = cv2.resize(mask, (frame_shape[1], frame_shape[0]))
    mask_uint8 = (mask_resized * 255).astype(np.uint8)

    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return {"has": False}

    cnt = max(contours, key=cv2.contourArea)
    if len(cnt) < 5:
        return {"has": False}

    sources = {}

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w_rect, h_rect), _ = rect
    box_points = cv2.boxPoints(rect).astype(np.float32)

    edge_a = box_points[1] - box_points[0]
    edge_b = box_points[2] - box_points[1]
    long_vec = edge_a if np.linalg.norm(edge_a) >= np.linalg.norm(edge_b) else edge_b
    angle_rect = np.degrees(np.arctan2(long_vec[1], long_vec[0]))
    if angle_rect < 0:
        angle_rect += 180.0
    sources["min_rect"] = angle_rect

    coords = cnt.reshape(-1, 2).astype(np.float32)
    pca_result = cv2.PCACompute2(coords, mean=None)
    mean, eigenvectors = pca_result[0], pca_result[1]
    major_vec = eigenvectors[0]
    angle_pca = np.degrees(np.arctan2(major_vec[1], major_vec[0]))
    if angle_pca < 0:
        angle_pca += 180.0
    sources["pca"] = float(angle_pca)

    ellipse = cv2.fitEllipse(cnt)
    angle_ellipse = ellipse[2]
    if angle_ellipse < 0:
        angle_ellipse += 180.0
    sources["ellipse"] = angle_ellipse

    angle = angle_average_deg(list(sources.values()))
    if angle is None:
        return {"has": False}

    tilt = angle - 90.0
    if tilt >= 90.0:
        tilt -= 180.0
    elif tilt < -90.0:
        tilt += 180.0

    rotation = -tilt  # rotate CCW to cancel tilt-from-vertical

    return {
        "has": True,
        "center": (cx, cy),
        "angle": angle,
        "rotation": rotation,
        "tilt": tilt,
        "box_points": box_points.astype(int),
        "mask_uint8": mask_uint8,
        "sources": sources,
    }


def rotate_crop_upright(crop, rotation_angle):
    if crop is None or crop.size == 0:
        return crop

    h, w = crop.shape[:2]
    if h == 0 or w == 0:
        return crop

    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, -rotation_angle, 1.0)

    cos = abs(M[0, 0])
    sin = abs(M[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]

    return cv2.warpAffine(crop, M, (new_w, new_h))


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

            if name in ["person", "keyboard", "laptop", "dining table"]:
                continue

            detection_count += 1

            if track_id not in tracked_objects:
                tracked_objects[track_id] = {
                    "initial_class": name,
                    "initial_conf": conf,
                    "frames_tracked": 0,
                    "low_conf": 0,
                }

            tracked_objects[track_id]["frames_tracked"] += 1
            tracked_objects[track_id]["last_conf"] = conf
            tracked_objects[track_id]["current_class"] = name

            warning = ""
            box_color = (0, 255, 0)

            if conf < 0.3:
                warning = " [LOW CONF!]"
                box_color = (0, 165, 255)
                tracked_objects[track_id]["low_conf"] += 1
            elif conf < 0.5:
                warning = " [uncertain]"
                box_color = (0, 255, 255)

            orientation = {"has": False}
            if masks is not None:
                mask = masks.data[i].cpu().numpy()
                orientation = compute_orientation(mask, frame.shape)

            crop_margin = 20
            cx1 = max(0, int(x1) - crop_margin)
            cy1 = max(0, int(y1) - crop_margin)
            cx2 = min(frame.shape[1], int(x2) + crop_margin)
            cy2 = min(frame.shape[0], int(y2) + crop_margin)
            crop = frame[cy1:cy2, cx1:cx2]

            crop_upright = crop
            rotation_used = 0.0

            if orientation.get("has") and crop.size > 0 and abs(orientation["rotation"]) > 1.0:
                crop_upright = rotate_crop_upright(crop, orientation["rotation"])
                rotation_used = orientation["rotation"]

            if show_crops and crop.size > 0:
                display_raw = crop.copy()
                display_rot = crop_upright.copy() if crop_upright is not None else crop.copy()

                cv2.putText(
                    display_raw,
                    f"ID:{track_id} raw",
                    (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                cv2.putText(
                    display_rot,
                    f"upright rot:{rotation_used:.1f}°",
                    (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )

                pad = 10
                h_max = max(display_raw.shape[0], display_rot.shape[0])
                spacer = np.zeros((h_max, pad, 3), dtype=np.uint8)
                debug_crops.append(np.hstack([display_raw, spacer, display_rot]))

            if (
                orientation.get("has")
                and name != tracked_objects[track_id]["initial_class"]
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
                        warning += f" (was {tracked_objects[track_id]['initial_class']}, crop says {crop_name})"
                        box_color = (0, 0, 255)

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), box_color, 2)
            label = f"{name} {conf:.2f} ID:{track_id}{warning}"
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            if orientation.get("has"):
                box_pts = orientation["box_points"]
                cv2.drawContours(frame, [box_pts], 0, (255, 0, 0), 2)

                cx, cy = orientation["center"]
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)

                ang_rad = np.deg2rad(orientation["angle"])
                length = 60
                ex = int(cx + length * np.cos(ang_rad))
                ey = int(cy + length * np.sin(ang_rad))
                cv2.arrowedLine(frame, (int(cx), int(cy)), (ex, ey), (255, 0, 0), 2)

                tilt = orientation["tilt"]
                cv2.putText(
                    frame,
                    f"angle:{orientation['angle']:.1f}° tilt:{tilt:.1f}°",
                    (int(cx) + 10, int(cy) + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 0),
                    1,
                )

                src_info = orientation["sources"]
                debug_txt = f"rect:{src_info['min_rect']:.1f} pca:{src_info['pca']:.1f} ell:{src_info['ellipse']:.1f}"
                cv2.putText(
                    frame,
                    debug_txt,
                    (int(cx) + 10, int(cy) + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (200, 200, 0),
                    1,
                )

    cv2.putText(frame, f"Detected: {detection_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if show_debug:
        y = 60
        cv2.putText(
            frame,
            f"Debug: ON | Crops:{show_crops}",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
        y += 20

        for tid, info in tracked_objects.items():
            txt = f"ID:{tid} {info['initial_class']} frames:{info['frames_tracked']} low:{info['low_conf']}"
            cv2.putText(frame, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            y += 18

    cv2.imshow("simple2_detect", frame)

    if show_crops and debug_crops:
        crop_height = 160
        row = []
        for panel in debug_crops:
            h, w = panel.shape[:2]
            if h <= 0:
                continue
            scale = crop_height / h
            new_w = int(w * scale)
            row.append(cv2.resize(panel, (new_w, crop_height)))
        if row:
            display = np.hstack(row) if len(row) > 1 else row[0]
            cv2.imshow("Crops: raw vs upright", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("d"):
        show_debug = not show_debug
        print(f"Debug view: {'ON' if show_debug else 'OFF'}")
    elif key == ord("c"):
        show_crops = not show_crops
        print(f"Crop panel: {'ON' if show_crops else 'OFF'}")
        if not show_crops and cv2.getWindowProperty("Crops: raw vs upright", cv2.WND_PROP_VISIBLE) >= 0:
            cv2.destroyWindow("Crops: raw vs upright")

cap.release()
cv2.destroyAllWindows()
