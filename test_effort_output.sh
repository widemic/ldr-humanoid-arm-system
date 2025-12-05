#!/bin/bash

# Monitor the effort command values being written by our controller
echo "Checking if controller is writing effort commands..."
echo "Looking for debug output from controller..."
echo ""

# Send a command and watch for controller debug output
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]" &

# Give it time to process
sleep 2

echo ""
echo "If you see 'ELBOW [Kp=4000]: error=..., effort=... Nm' above,"
echo "then the controller IS computing effort."
echo ""
echo "The problem is that GazeboSimSystem is ignoring our effort commands"
echo "and using the position command interface to do its own PD control."
