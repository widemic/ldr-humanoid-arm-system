#!/usr/bin/env python3
"""
Debug detection - shows each processing step
"""

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Adjustable parameters
min_area = 500  # Start MUCH lower
max_area = 200000

print("=" * 60)
print("Detection Debugger")
print("=" * 60)
print(f"Min area: {min_area} (press 'a' to decrease, 's' to increase)")
print(f"Max area: {max_area}")
print("Press 'q' to quit")
print("=" * 60)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Adaptive threshold
    binary = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 21, 3)

    # Morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter by area
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            valid_contours.append(contour)

    # Draw on original
    vis = frame.copy()
    cv2.drawContours(vis, valid_contours, -1, (0, 255, 0), 2)

    # Stats
    stats = f"Found: {len(valid_contours)} objects | Min area: {min_area}"
    cv2.putText(vis, stats, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis, "Press 'a' to decrease min area, 's' to increase", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Show all stages
    cv2.imshow('1. Original', frame)
    cv2.imshow('2. Grayscale', gray)
    cv2.imshow('3. Binary', binary)
    cv2.imshow('4. Detected Objects', vis)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('a'):  # Decrease min_area
        min_area = max(100, min_area - 100)
        print(f"Min area: {min_area}")
    elif key == ord('s'):  # Increase min_area
        min_area += 100
        print(f"Min area: {min_area}")

cap.release()
cv2.destroyAllWindows()
