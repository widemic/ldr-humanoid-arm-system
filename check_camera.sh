#!/bin/bash
# Script to check camera topics and visualize camera data

echo "=== Checking Camera Topics ==="
echo ""
echo "Available camera topics:"
ros2 topic list | grep camera

echo ""
echo "=== Camera Topic Info ==="
echo ""
echo "Color image topic:"
ros2 topic info /camera/color/image_raw 2>/dev/null || echo "  Not available yet"

echo ""
echo "Depth image topic:"
ros2 topic info /camera/depth/image_raw 2>/dev/null || echo "  Not available yet"

echo ""
echo "Point cloud topic:"
ros2 topic info /camera/points 2>/dev/null || echo "  Not available yet"

echo ""
echo "=== To visualize camera data in RViz, run: ==="
echo "ros2 run rviz2 rviz2"
echo ""
echo "Then add these display types:"
echo "  - Image display for /camera/color/image_raw (color image)"
echo "  - Image display for /camera/depth/image_raw (depth image)"
echo "  - PointCloud2 display for /camera/points (point cloud)"
echo ""
echo "=== To echo a single image (color): ==="
echo "ros2 topic echo /camera/color/image_raw --once"
echo ""
echo "=== To echo a single depth image: ==="
echo "ros2 topic echo /camera/depth/image_raw --once"
