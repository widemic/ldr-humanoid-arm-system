#!/usr/bin/env python3

import cv2


def main():
    """Simple webcam viewer using OpenCV."""

    # Open webcam (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    print("Webcam viewer started. Press 'q' to quit.")

    while True:
        # Capture frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame")
            break

        # Display frame
        cv2.imshow('Webcam Viewer', frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Webcam viewer closed")


if __name__ == '__main__':
    main()
