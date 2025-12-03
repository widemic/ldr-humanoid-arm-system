#!/usr/bin/env python3
"""
Standalone Webcam Grasp Detector (No ROS Topics - Direct OpenCV Window)
Perfect for quick testing without needing ROS topic visualization tools.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import sys


class GraspPoint:
    def __init__(self):
        self.pixel_location: Tuple[int, int] = (0, 0)
        self.orientation: float = 0.0
        self.grasp_type: str = "unknown"
        self.confidence: float = 0.0


class DetectedObject:
    def __init__(self):
        self.contour: np.ndarray = None
        self.center: Tuple[int, int] = (0, 0)
        self.orientation: float = 0.0
        self.bounding_box = None
        self.area: float = 0.0
        self.grasp_points: List[GraspPoint] = []
        self.object_type: str = "unknown"


class StandaloneGraspDetector:
    def __init__(self, webcam_device=0, min_area=2000, max_area=150000):
        self.min_area = min_area
        self.max_area = max_area

        # Open webcam
        self.cap = cv2.VideoCapture(webcam_device)
        if not self.cap.isOpened():
            print(f"ERROR: Cannot open webcam {webcam_device}")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("=" * 60)
        print("Webcam Grasp Detector - Standalone Mode")
        print("=" * 60)
        print(f"Webcam device: {webcam_device}")
        print(f"Object area range: {min_area} - {max_area} pixels")
        print("\nControls:")
        print("  - Press 'q' to quit")
        print("  - Press '+' to increase min area (less sensitive)")
        print("  - Press '-' to decrease min area (more sensitive)")
        print("=" * 60)

    def run(self):
        """Main processing loop."""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Detect objects
            objects = self.detect_objects(frame)

            # Compute grasps
            for obj in objects:
                self.compute_grasp_points(obj)

            # Visualize
            vis = self.visualize_detections(frame.copy(), objects)

            # Show stats
            stats_text = f"Objects: {len(objects)} | Min Area: {self.min_area}"
            cv2.putText(vis, stats_text, (10, vis.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Display
            cv2.imshow('Grasp Detection', vis)

            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('+') or key == ord('='):
                self.min_area = min(self.min_area + 500, 50000)
                print(f"Min area increased to {self.min_area}")
            elif key == ord('-') or key == ord('_'):
                self.min_area = max(self.min_area - 500, 100)
                print(f"Min area decreased to {self.min_area}")

        self.cap.release()
        cv2.destroyAllWindows()

    def detect_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """Detect objects via contours."""
        objects = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 3)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            obj = DetectedObject()
            obj.contour = contour
            obj.area = area

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            obj.center = (cx, cy)

            if len(contour) >= 5:
                obj.bounding_box = cv2.minAreaRect(contour)
                obj.orientation = np.deg2rad(obj.bounding_box[2])

            obj.object_type = self.classify_shape(contour, obj.bounding_box)
            objects.append(obj)

        return objects

    def classify_shape(self, contour, bbox) -> str:
        """Classify object shape."""
        if bbox is None:
            return "unknown"

        width, height = bbox[1]
        if height == 0:
            return "unknown"

        aspect_ratio = max(width, height) / min(width, height)
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return "unknown"

        circularity = 4 * np.pi * area / (perimeter ** 2)

        if circularity > 0.8:
            return "circular"
        elif aspect_ratio > 3.0:
            return "elongated"
        elif aspect_ratio < 1.5 and circularity > 0.6:
            return "square"
        else:
            return "irregular"

    def compute_grasp_points(self, obj: DetectedObject):
        """Compute all grasp points."""
        grasp_points = []

        # Center grasp
        center_grasp = GraspPoint()
        center_grasp.pixel_location = obj.center
        center_grasp.orientation = obj.orientation
        center_grasp.grasp_type = "center"
        center_grasp.confidence = 0.8
        grasp_points.append(center_grasp)

        # Edge grasps (elongated)
        if obj.object_type == "elongated" and obj.bounding_box:
            width, height = obj.bounding_box[1]
            center = obj.bounding_box[0]
            angle = np.deg2rad(obj.bounding_box[2])

            if width > height:
                offset = width / 4
                for sign in [-1, 1]:
                    px = int(center[0] + sign * offset * np.cos(angle))
                    py = int(center[1] + sign * offset * np.sin(angle))

                    grasp = GraspPoint()
                    grasp.pixel_location = (px, py)
                    grasp.orientation = angle + np.pi/2
                    grasp.grasp_type = "edge"
                    grasp.confidence = 0.7
                    grasp_points.append(grasp)

        # Corner grasps
        if obj.object_type in ["square", "irregular"] and obj.bounding_box:
            box_points = cv2.boxPoints(obj.bounding_box)
            for point in box_points:
                px, py = int(point[0]), int(point[1])

                grasp = GraspPoint()
                grasp.pixel_location = (px, py)
                dx = obj.center[0] - px
                dy = obj.center[1] - py
                grasp.orientation = np.arctan2(dy, dx)
                grasp.grasp_type = "corner"
                grasp.confidence = 0.6
                grasp_points.append(grasp)

        # Handle detection
        hull = cv2.convexHull(obj.contour, returnPoints=False)
        if len(hull) > 3 and len(obj.contour) > 3:
            defects = cv2.convexityDefects(obj.contour, hull)
            if defects is not None:
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]
                    depth = d / 256.0

                    if depth > 15:
                        far = tuple(obj.contour[f][0])
                        grasp = GraspPoint()
                        grasp.pixel_location = far
                        grasp.orientation = obj.orientation
                        grasp.grasp_type = "handle"
                        grasp.confidence = 0.5 + min(depth / 50.0, 0.4)
                        grasp_points.append(grasp)

        obj.grasp_points = grasp_points

    def visualize_detections(self, image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
        """Draw detections on image."""
        vis = image.copy()

        color_map = {
            "center": (255, 255, 0),    # Cyan
            "edge": (255, 0, 255),      # Magenta
            "corner": (0, 255, 255),    # Yellow
            "handle": (255, 128, 0),    # Orange
        }

        for obj in objects:
            # Contour
            cv2.drawContours(vis, [obj.contour], -1, (0, 255, 0), 2)

            # Bounding box
            if obj.bounding_box:
                box = cv2.boxPoints(obj.bounding_box)
                box = np.int0(box)
                cv2.drawContours(vis, [box], 0, (255, 0, 0), 2)

            # Center
            cv2.circle(vis, obj.center, 5, (0, 0, 255), -1)

            # Grasp points
            for grasp in obj.grasp_points:
                px, py = grasp.pixel_location
                color = color_map.get(grasp.grasp_type, (255, 255, 255))

                cv2.circle(vis, (px, py), 8, color, 2)

                # Orientation arrow
                length = 40
                end_x = int(px + length * np.cos(grasp.orientation))
                end_y = int(py + length * np.sin(grasp.orientation))
                cv2.arrowedLine(vis, (px, py), (end_x, end_y), color, 2)

                # Confidence
                cv2.putText(vis, f'{grasp.confidence:.2f}', (px + 10, py - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Object info
            info = f'{obj.object_type} ({obj.area:.0f}px)'
            cv2.putText(vis, info, (obj.center[0] - 40, obj.center[1] - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Legend
        y = 30
        cv2.putText(vis, "Cyan=Center, Magenta=Edge, Yellow=Corner, Orange=Handle",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return vis


def main():
    # Parse command line args
    webcam_device = 0
    min_area = 2000
    max_area = 150000

    if len(sys.argv) > 1:
        webcam_device = int(sys.argv[1])
    if len(sys.argv) > 2:
        min_area = int(sys.argv[2])

    detector = StandaloneGraspDetector(webcam_device, min_area, max_area)
    detector.run()


if __name__ == '__main__':
    main()
