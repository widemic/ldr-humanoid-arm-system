#!/usr/bin/env python3

import cv2
import numpy as np


class WebcamObjectTracker:
    """Tracks multiple objects from webcam feed, excluding humans."""

    def __init__(self):
        self.cap = None
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Detection parameters
        self.min_object_area = 500
        self.color_lower = np.array([0, 50, 50])
        self.color_upper = np.array([180, 255, 255])

        # Trail parameters
        self.max_trail_length = 30
        self.object_trails = {}

    def initialize_camera(self):
        """Initialize webcam capture."""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam")

    def detect_humans(self, frame):
        """Detect human regions in frame."""
        humans, _ = self.hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05
        )
        return humans

    def create_human_exclusion_mask(self, frame_shape, humans):
        """Create mask that excludes human regions."""
        mask = np.ones(frame_shape[:2], dtype=np.uint8) * 255
        for (x, y, w, h) in humans:
            cv2.rectangle(mask, (x, y), (x + w, y + h), 0, -1)
        return mask

    def detect_objects(self, frame, exclusion_mask):
        """Detect objects using color segmentation."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        object_mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        object_mask = cv2.bitwise_and(object_mask, object_mask, mask=exclusion_mask)

        contours, _ = cv2.findContours(
            object_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return contours, object_mask

    def filter_objects(self, contours):
        """Filter contours by minimum area."""
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_object_area:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                objects.append({
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'area': area
                })
        return objects

    def update_trails(self, objects):
        """Update object trails based on current detections."""
        # Match objects to existing trails based on proximity
        current_centers = [obj['center'] for obj in objects]

        # Remove old trails if no nearby objects
        trails_to_remove = []
        for trail_id in list(self.object_trails.keys()):
            trail = self.object_trails[trail_id]
            if len(current_centers) == 0:
                trails_to_remove.append(trail_id)
                continue

            # Check if trail has a nearby object
            last_pos = trail[-1]
            distances = [np.linalg.norm(np.array(last_pos) - np.array(center))
                        for center in current_centers]
            if min(distances) > 100:  # No object within 100 pixels
                trails_to_remove.append(trail_id)

        for trail_id in trails_to_remove:
            del self.object_trails[trail_id]

        # Update or create trails
        used_centers = set()
        for trail_id, trail in list(self.object_trails.items()):
            if len(current_centers) == 0:
                break

            # Find closest object to this trail
            last_pos = trail[-1]
            distances = [(i, np.linalg.norm(np.array(last_pos) - np.array(center)))
                        for i, center in enumerate(current_centers) if i not in used_centers]

            if distances:
                closest_idx, dist = min(distances, key=lambda x: x[1])
                if dist < 100:  # Object is close enough
                    trail.append(current_centers[closest_idx])
                    if len(trail) > self.max_trail_length:
                        trail.pop(0)
                    used_centers.add(closest_idx)

        # Create new trails for unmatched objects
        for i, center in enumerate(current_centers):
            if i not in used_centers:
                new_trail_id = max(self.object_trails.keys()) + 1 if self.object_trails else 0
                self.object_trails[new_trail_id] = [center]

    def draw_trails(self, frame):
        """Draw motion trails for tracked objects."""
        for trail in self.object_trails.values():
            if len(trail) > 1:
                # Draw lines connecting trail points
                for i in range(1, len(trail)):
                    # Fade color based on age
                    alpha = i / len(trail)
                    color = (int(255 * alpha), int(165 * alpha), 0)  # Orange gradient
                    thickness = max(1, int(3 * alpha))
                    cv2.line(frame, trail[i-1], trail[i], color, thickness)

    def draw_humans(self, frame, humans):
        """Draw bounding boxes around detected humans."""
        for (x, y, w, h) in humans:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(
                frame, "HUMAN (excluded)", (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

    def draw_objects(self, frame, objects):
        """Draw bounding boxes and info for tracked objects."""
        for i, obj in enumerate(objects):
            x, y, w, h = obj['bbox']
            center_x, center_y = obj['center']

            # Bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Center point
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # Object label
            text = f"Obj{i+1}: ({center_x},{center_y})"
            cv2.putText(
                frame, text, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

    def draw_summary(self, frame, num_objects, num_humans):
        """Draw summary information on frame."""
        summary = f"Objects tracked: {num_objects} | Humans excluded: {num_humans}"
        cv2.putText(
            frame, summary, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
        )

    def run(self):
        """Main tracking loop."""
        self.initialize_camera()

        print("Webcam multi-object tracker started.")
        print("Tracking objects, excluding humans")
        print("Press 'q' to quit")

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Detect humans and create exclusion mask
                humans = self.detect_humans(frame)
                exclusion_mask = self.create_human_exclusion_mask(frame.shape, humans)

                # Detect and filter objects
                contours, object_mask = self.detect_objects(frame, exclusion_mask)
                objects = self.filter_objects(contours)

                # Update and draw trails
                self.update_trails(objects)
                self.draw_trails(frame)

                # Draw results
                self.draw_humans(frame, humans)
                self.draw_objects(frame, objects)
                self.draw_summary(frame, len(objects), len(humans))

                # Display
                cv2.imshow('Webcam Multi-Object Tracker', frame)
                cv2.imshow('Object Mask', object_mask)

                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

    def cleanup(self):
        """Release resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Webcam object tracker closed")


def main():
    tracker = WebcamObjectTracker()
    tracker.run()


if __name__ == '__main__':
    main()
