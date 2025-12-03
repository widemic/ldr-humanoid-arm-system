#!/bin/bash
# Ultra-simple test - no ROS needed, just OpenCV window

echo "=========================================="
echo "Simple Grasp Detection Test (No ROS)"
echo "=========================================="
echo ""
echo "This opens a direct OpenCV window showing grasp detection."
echo "No ROS topics, no extra tools needed!"
echo ""
echo "Controls:"
echo "  - Press 'q' to quit"
echo "  - Press '+' to increase sensitivity"
echo "  - Press '-' to decrease sensitivity"
echo ""
echo "Press Ctrl+C to stop or 'q' in the window"
echo ""

# Just run the Python script directly
python3 src/perception/perception_tests/scripts/webcam_grasp_standalone.py
