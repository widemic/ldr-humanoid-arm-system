#!/bin/bash
# Launch ONLY object tracking (NO OctoMap - eliminates ghost objects)

echo "=========================================="
echo "  Dynamic Object Tracking ONLY"
echo "  (No OctoMap - No Ghost Objects!)"
echo "=========================================="
echo ""

# Source workspace
source install/setup.bash

echo "Launching object tracker with optimized settings..."
echo "  - OctoMap: DISABLED"
echo "  - Object Tracking: ENABLED"
echo "  - Min cluster size: 100 points"
echo "  - Cluster tolerance: 0.15m (good for humans)"
echo "  - Min object height: 0.05m (filters ground plane)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch with optimized parameters for human/complex objects
ros2 launch arm_perception perception.launch.py \
    enable_octomap:=false \
    enable_object_tracking:=true \
    use_sim_time:=true

# Alternative: run tracker directly for more debugging
# ros2 run arm_perception dynamic_object_tracker.py \
#     --ros-args \
#     --log-level info \
#     --param use_sim_time:=true \
#     --param min_cluster_size:=100 \
#     --param cluster_tolerance:=0.15 \
#     --param min_object_height:=0.05 \
#     --param max_tracking_distance:=0.5
