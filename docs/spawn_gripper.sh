#!/bin/bash
# Spawn hand_controller for gripper control
# Run this while the system is running if the controller didn't spawn automatically

echo "Spawning hand_controller..."
echo ""

# Simple spawn - configuration is loaded from Gazebo's controllers.yaml
ros2 run controller_manager spawner hand_controller \
    --controller-manager /controller_manager

echo ""
echo "Controller status:"
ros2 control list_controllers
