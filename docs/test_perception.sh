#!/bin/bash
# Quick test script for perception system

echo "=== Perception System Test ==="
echo ""

echo "1. Checking if point cloud topic exists..."
ros2 topic list | grep "/camera/depth/points"
if [ $? -eq 0 ]; then
    echo "  ✓ Point cloud topic found"
else
    echo "  ✗ Point cloud topic NOT found - is Gazebo running?"
    exit 1
fi

echo ""
echo "2. Checking point cloud data rate..."
timeout 3 ros2 topic hz /camera/depth/points 2>&1 | grep "average rate"

echo ""
echo "3. Checking point cloud size..."
ros2 topic echo /camera/depth/points --field width --once
ros2 topic echo /camera/depth/points --field height --once

echo ""
echo "4. Checking if dynamic_object_tracker is running..."
ros2 node list | grep dynamic_object_tracker
if [ $? -eq 0 ]; then
    echo "  ✓ Tracker node is running"
else
    echo "  ✗ Tracker node NOT running"
fi

echo ""
echo "5. Checking tracked_objects topic..."
timeout 2 ros2 topic hz /tracked_objects 2>&1 | grep "average rate" || echo "  No data on /tracked_objects yet"

echo ""
echo "6. Checking planning_scene updates..."
timeout 2 ros2 topic hz /planning_scene 2>&1 | grep "average rate" || echo "  No data on /planning_scene yet"

echo ""
echo "=== Test Complete ==="
echo ""
echo "To see tracker logs:"
echo "  ros2 node info /dynamic_object_tracker"
echo ""
echo "To see real-time output:"
echo "  ros2 topic echo /tracked_objects"
