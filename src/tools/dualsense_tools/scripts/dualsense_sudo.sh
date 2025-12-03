#!/bin/bash
# Simple wrapper to run dualsense_node with sudo while preserving ROS environment

# Source the ROS setup if it exists
if [ -f "${ROS_SETUP}" ]; then
    source "${ROS_SETUP}"
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Run with sudo, preserving environment variables
sudo -E $(which python3) $(ros2 pkg prefix dualsense_tools)/lib/dualsense_tools/dualsense_node.py "$@"
