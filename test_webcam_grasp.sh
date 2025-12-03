#!/bin/bash
# Quick test script for webcam grasp detector

echo "=========================================="
echo "Webcam Grasp Detection Test"
echo "=========================================="
echo ""
echo "This will launch the webcam grasp detector."
echo "Place objects in front of your webcam to detect grasp points."
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Source ROS 2 workspace
source install/setup.bash

# Run the webcam grasp detector
ros2 run perception_tests webcam_grasp_detector.py --ros-args \
  -p webcam_device:=0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p assumed_depth:=0.5 \
  -p min_object_area:=2000 \
  -p max_object_area:=150000
