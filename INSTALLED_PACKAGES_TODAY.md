# ROS 2 Packages Installed Today

## Core ROS 2 Jazzy Packages

### Control & Controllers
```bash
sudo apt install -y ros-jazzy-ros2-control
sudo apt install -y ros-jazzy-ros2-controllers
sudo apt install -y ros-jazzy-joint-state-broadcaster
sudo apt install -y ros-jazzy-joint-trajectory-controller
sudo apt install -y ros-jazzy-gz-ros2-control
```

### MoveIt Motion Planning
```bash
sudo apt install -y ros-jazzy-moveit-configs-utils
sudo apt install -y ros-jazzy-moveit-ros-visualization
sudo apt install -y ros-jazzy-moveit-ros-perception
sudo apt install -y ros-jazzy-moveit-setup-assistant
```

### Gazebo Integration
```bash
sudo apt install -y ros-jazzy-ros-gz-sim
sudo apt install -y ros-jazzy-ros-gz-bridge
```

### Vision & Perception
```bash
sudo apt install -y ros-jazzy-vision-opencv
sudo apt install -y ros-jazzy-image-tools
```

### OctoMap (3D Mapping)
```bash
sudo apt install -y ros-jazzy-octomap
sudo apt install -y ros-jazzy-octomap-msgs
sudo apt install -y ros-jazzy-octomap-mapping
sudo apt install -y ros-jazzy-octomap-server
sudo apt install -y ros-jazzy-octomap-rviz-plugins
```

### TF2 Transformations
```bash
sudo apt install -y ros-jazzy-tf2-sensor-msgs
```

### Utilities
```bash
sudo apt install -y ros-jazzy-ros2launch
sudo apt install -y ros-jazzy-joy
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
```

## Python Packages

### Scientific Computing (for Perception)
```bash
sudo apt install python3-sklearn python3-numpy
```

---

## Quick Reinstall Script

If you need to reinstall everything on a new machine:

```bash
#!/bin/bash
# ROS 2 Jazzy packages for LDR Humanoid Arm System

# Control & Controllers
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-gz-ros2-control

# MoveIt
sudo apt install -y \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-ros-perception \
    ros-jazzy-moveit-setup-assistant

# Gazebo
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge

# Vision
sudo apt install -y \
    ros-jazzy-vision-opencv \
    ros-jazzy-image-tools

# OctoMap
sudo apt install -y \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-mapping \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap-rviz-plugins

# TF2 & Utilities
sudo apt install -y \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-ros2launch \
    ros-jazzy-joy \
    ros-jazzy-rmw-cyclonedds-cpp

# Python packages
sudo apt install -y \
    python3-sklearn \
    python3-numpy
```

## Total Packages Installed

**Main packages:** 18 ROS 2 packages + 2 Python packages
**Automatic dependencies:** ~40+ additional packages installed automatically

## Usage

All these packages are now available in your ROS 2 Jazzy workspace and enable:
- ✅ Robot control with ros2_control
- ✅ Motion planning with MoveIt2
- ✅ Gazebo Harmonic simulation
- ✅ 3D perception with OctoMap
- ✅ Computer vision with OpenCV
- ✅ Point cloud processing
- ✅ Object detection and classification
