# 3D Perception - Quick Start Guide

## What This Does

Automatically detects **cylinders** and **boxes** from the camera and adds them as **collision objects** in MoveIt planning scene.

**Inspired by:** [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2/tree/jazzy)

## Installation

```bash
# Install Python dependencies
pip3 install scikit-learn numpy

# Build the package
cd ~/ros2_ws/ldr-humanoid-arm-system
colcon build --packages-select arm_perception
source install/setup.bash
```

## Usage

### Simple 2-Terminal Launch

```bash
# Terminal 1: Launch Gazebo + Robot + MoveIt
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Terminal 2: Launch perception (wait 10 seconds after Terminal 1)
ros2 launch arm_perception perception.launch.py
```

### What You'll See

1. **In Gazebo:** Robot and camera view
2. **In RViz:**
   - **Green cylinders** - Detected cylindrical objects
   - **Blue boxes** - Detected box objects
   - MoveIt planning scene shows collision objects

## How It Works

```
Camera sees objects → Detects shapes → Adds to MoveIt scene
```

**Processing:**
1. Get point cloud from camera (`/camera/depth/points`)
2. Remove table/floor using RANSAC plane segmentation
3. Cluster remaining points (DBSCAN)
4. Detect shape type (cylinder vs box) per cluster
5. Publish to MoveIt as collision objects (`/collision_object`)
6. Visualize in RViz (`/perception/detected_objects`)

## Testing

### Add Test Objects in Gazebo

In Gazebo GUI:
1. Click **Insert** tab
2. Add **Cylinder** or **Box**
3. Place in camera view
4. Watch it appear in RViz as colored marker
5. See it in MoveIt planning scene

### Verify It's Working

```bash
# Check objects are being detected
ros2 topic echo /collision_object

# Check visualization markers
ros2 topic hz /perception/detected_objects

# Should see logs like:
# [INFO] [perception_node]: Found 2 clusters
# [INFO] [perception_node]: Published 2 collision objects
```

## Configuration

Edit [config/perception.yaml](src/perception/arm_perception/config/perception.yaml) to adjust:

```yaml
# Minimum object size (in meters)
min_object_height: 0.02  # 2cm

# Cylinder radius range
cylinder_radius_min: 0.01  # 1cm
cylinder_radius_max: 0.15  # 15cm

# Processing speed
processing_rate: 1.0  # 1 Hz (once per second)

# Detection sensitivity
min_cluster_size: 50  # Fewer points = more sensitive
```

## RViz Setup

Add these displays:

1. **MarkerArray**:
   - Topic: `/perception/detected_objects`
   - Shows green cylinders and blue boxes

2. **Planning Scene** (MoveIt plugin):
   - Automatically shows collision objects

## Algorithms Used

- **RANSAC** - Plane segmentation (remove table)
- **DBSCAN** - Point clustering (group objects)
- **Circularity Analysis** - Detect cylinders vs boxes
- **Bounding Box** - Compute dimensions

## Topics

### Subscribed:
- `/camera/depth/points` - RGBD point cloud

### Published:
- `/collision_object` - MoveIt collision objects
- `/perception/detected_objects` - RViz markers

## Troubleshooting

### No detections

```bash
# 1. Check camera is publishing
ros2 topic hz /camera/depth/points

# 2. Check TF is working
ros2 run tf2_ros tf2_echo base_fixture_link camera_link

# 3. Check perception node is running
ros2 node list | grep perception
```

### Too many false detections

Edit `config/perception.yaml`:
```yaml
min_cluster_size: 100  # Increase (was 50)
min_object_height: 0.05  # Increase (was 0.02)
```

### Missing small objects

```yaml
min_cluster_size: 30  # Decrease
min_object_height: 0.01  # Decrease
```

## What's Different from mycobot_ros2

| mycobot_ros2 | This implementation |
|-------------|---------------------|
| C++ | **Python** (easier to modify) |
| PCL library | **sklearn** (simpler) |
| Hough Transform | **Circularity analysis** |
| Complex | **Streamlined** |

## Full Documentation

See [PERCEPTION_SYSTEM_GUIDE.md](PERCEPTION_SYSTEM_GUIDE.md) for:
- Complete algorithm details
- Advanced configuration
- Code structure
- Performance tuning
- Future improvements

---

## 🚀 Quick Demo

```bash
# Build
colcon build --packages-select arm_perception && source install/setup.bash

# Launch system
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# In another terminal (wait 10s):
ros2 launch arm_perception perception.launch.py

# Add objects in Gazebo and watch them appear in RViz!
```

**Detection Rate:** ~1 Hz (once per second)
**Shapes Detected:** Cylinders (green), Boxes (blue)
**Integration:** Automatic MoveIt collision objects
