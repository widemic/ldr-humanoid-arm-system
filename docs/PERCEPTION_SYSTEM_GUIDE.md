# 3D Perception System for MoveIt

## Overview

The **arm_perception** package provides automatic 3D object detection and collision object generation for MoveIt from RGBD camera data. It detects geometric shapes (cylinders and boxes) in the Gazebo simulation and automatically adds them to the MoveIt planning scene.

**Inspired by:** [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2/tree/jazzy)

## Features

✅ **Automatic shape detection** - Detects cylinders and boxes from point cloud data
✅ **RANSAC plane segmentation** - Separates support surface (table) from objects
✅ **DBSCAN clustering** - Groups points into individual objects
✅ **MoveIt integration** - Automatically publishes collision objects to planning scene
✅ **Real-time visualization** - Shows detected objects as RViz markers
✅ **Configurable parameters** - Adjust detection thresholds via YAML config

## How It Works

### Processing Pipeline

```
Camera Point Cloud (/camera/depth/points)
    ↓
Transform to base_fixture_link
    ↓
Filter by Z bounds (-0.5m to 1.5m)
    ↓
RANSAC Plane Segmentation (detect table)
    ↓
DBSCAN Clustering (group remaining points)
    ↓
Shape Detection per Cluster:
  - Circularity analysis → Cylinder
  - Bounding box → Box
    ↓
Publish to MoveIt Planning Scene (/collision_object)
    ↓
Visualize in RViz (/perception/detected_objects)
```

### Shape Detection Algorithms

**Cylinder Detection:**
- Projects cluster to 2D (x, y plane)
- Computes centroid and point distances
- If distances have low std deviation → circular cross-section
- Validates radius is within range (1cm - 15cm)
- Creates cylinder with detected radius and cluster height

**Box Detection:**
- Computes 3D bounding box of cluster
- Uses min/max coordinates for dimensions
- Creates box primitive with computed dimensions

## Installation & Dependencies

### Required Packages

```bash
# ROS 2 Jazzy packages (should already be installed)
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions

# Python dependencies
pip3 install scikit-learn numpy
```

### Build

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
colcon build --packages-select arm_perception
source install/setup.bash
```

## Usage

### Standalone Launch

```bash
# Launch perception node alone (requires camera and TF to be running)
ros2 launch arm_perception perception.launch.py
```

### Integrated with Full System

```bash
# Terminal 1: Launch Gazebo + Robot + MoveIt
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Terminal 2: Launch perception (wait 10s for system to initialize)
ros2 launch arm_perception perception.launch.py
```

### With OctoMap

```bash
# Terminal 1: Full system with OctoMap
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py

# Terminal 2: Perception
ros2 launch arm_perception perception.launch.py
```

## Configuration

**File:** [config/perception.yaml](src/perception/arm_perception/config/perception.yaml)

### Key Parameters

```yaml
perception_node:
  ros__parameters:
    # Input
    camera_topic: "/camera/depth/points"
    target_frame: "base_fixture_link"

    # Plane segmentation (RANSAC)
    plane_distance_threshold: 0.01  # 1cm tolerance

    # Clustering (DBSCAN)
    cluster_tolerance: 0.02         # 2cm - proximity threshold
    min_cluster_size: 50            # Minimum points per object
    max_cluster_size: 5000          # Maximum points per object

    # Shape detection
    cylinder_radius_min: 0.01       # 1cm minimum radius
    cylinder_radius_max: 0.15       # 15cm maximum radius
    min_object_height: 0.02         # 2cm minimum height

    # Processing
    processing_rate: 1.0            # Process at 1 Hz

    # Z filtering (in target_frame)
    z_min: -0.5                     # Filter below -0.5m
    z_max: 1.5                      # Filter above 1.5m
```

### Tuning Tips

**Too many false detections:**
- Increase `min_cluster_size` (e.g., 100)
- Increase `min_object_height` (e.g., 0.05)
- Decrease `z_max` to limit view distance

**Missing small objects:**
- Decrease `min_cluster_size` (e.g., 30)
- Decrease `min_object_height` (e.g., 0.01)
- Decrease `cluster_tolerance` for tighter grouping

**Cylinders detected as boxes:**
- Increase `cylinder_radius_max`
- Check that circularity threshold in code suits your objects

## ROS 2 Topics

### Subscribed

- `/camera/depth/points` (sensor_msgs/PointCloud2) - RGBD camera point cloud

### Published

- `/collision_object` (moveit_msgs/CollisionObject) - Detected objects for MoveIt planning scene
- `/perception/detected_objects` (visualization_msgs/MarkerArray) - Visualization markers for RViz

## Visualization in RViz

Add these displays in RViz:

1. **MarkerArray** display:
   - Topic: `/perception/detected_objects`
   - Shows detected objects:
     - **Green cylinders** - Detected cylinders
     - **Blue boxes** - Detected boxes

2. **Planning Scene** display (MoveIt):
   - Shows collision objects in planning scene
   - Objects detected by perception appear here automatically

## Testing

### Add Test Objects in Gazebo

You can spawn test objects in Gazebo to verify detection:

```bash
# Spawn a cylinder
gz model --spawn-file=/path/to/cylinder.sdf --model-name=test_cylinder

# Or manually add objects using Gazebo GUI:
# Insert → Cylinder / Box
```

### Verify Detection

```bash
# Check if collision objects are being published
ros2 topic echo /collision_object

# Check visualization markers
ros2 topic echo /perception/detected_objects

# Monitor perception node logs
ros2 node list | grep perception
ros2 topic hz /collision_object
```

### Expected Output

When objects are detected, you'll see logs like:
```
[INFO] [perception_node]: Detected support plane with 1523 points
[INFO] [perception_node]: Found 2 clusters
[INFO] [perception_node]: Published 2 collision objects
```

## Troubleshooting

### No objects detected

**Check camera is publishing:**
```bash
ros2 topic hz /camera/depth/points
# Should show ~10-30 Hz
```

**Check TF transforms exist:**
```bash
ros2 run tf2_ros tf2_echo base_fixture_link camera_link
# Should show transform
```

**Verify Z filtering isn't too restrictive:**
- Check `z_min` and `z_max` parameters
- Objects must be within these bounds

### Objects detected but not in planning scene

**Check MoveIt is running:**
```bash
ros2 node list | grep move_group
# Should show /move_group
```

**Check collision object topic:**
```bash
ros2 topic info /collision_object
# Should show subscribers (move_group)
```

### Performance issues (slow processing)

**Reduce processing rate:**
```yaml
processing_rate: 0.5  # Process every 2 seconds instead of 1
```

**Reduce max cluster size:**
```yaml
max_cluster_size: 2000  # Process smaller clusters only
```

##  Code Structure

```
arm_perception/
├── config/
│   └── perception.yaml           # Configuration parameters
├── launch/
│   └── perception.launch.py      # Launch file
├── scripts/
│   └── perception_node.py        # Main perception node (Python)
├── CMakeLists.txt                # Build configuration
└── package.xml                   # Package manifest
```

### Main Components

**perception_node.py** (700 lines):
- `PerceptionNode` class - Main ROS 2 node
- `pointcloud_callback()` - Stores latest point cloud
- `process_latest_pointcloud()` - Main processing pipeline
- `segment_plane()` - RANSAC plane fitting
- `cluster_points()` - DBSCAN clustering
- `detect_shape()` - Shape detection (cylinder vs box)
- `publish_collision_objects()` - MoveIt collision object publishing
- `publish_visualization_markers()` - RViz marker publishing

## Algorithms

### RANSAC Plane Segmentation

Uses `sklearn.linear_model.RANSACRegressor` to fit plane equation:
```
z = ax + by + c
```

- **Min samples:** 3 points
- **Residual threshold:** 1cm (configurable)
- **Max trials:** 1000
- **Output:** Inliers (plane points) and outliers (object points)

### DBSCAN Clustering

Uses `sklearn.cluster.DBSCAN` for point grouping:
- **Epsilon:** 2cm (configurable `cluster_tolerance`)
- **Min samples:** 10 points
- **Output:** Individual object clusters

### Circularity Analysis

For each cluster:
1. Compute 2D centroid in x-y plane
2. Calculate distances from centroid to all points
3. Compute std deviation of distances
4. Circularity ratio = std / mean
5. If ratio < 0.3 → cylinder, else → box

## Comparison with mycobot_ros2

| Feature | mycobot_ros2 | arm_perception (this) |
|---------|--------------|----------------------|
| Language | C++ | Python |
| Plane Segmentation | PCL RANSAC | sklearn RANSAC |
| Clustering | PCL EuclideanClusterExtraction | sklearn DBSCAN |
| Shape Detection | Hough Transform + RANSAC | Circularity analysis |
| Dependencies | PCL, custom C++ | sklearn, numpy (lighter) |
| Performance | Faster (C++) | Simpler, easier to modify |

## Future Improvements

- [ ] Add C++ implementation for better performance
- [ ] Implement Hough Transform for more accurate cylinder detection
- [ ] Add sphere detection
- [ ] Integrate color information from RGB data
- [ ] Add object tracking (persistent IDs across frames)
- [ ] Add service interface (like mycobot's GetPlanningScene service)
- [ ] Add plane as collision object (table surface)

## Example Use Case: Pick and Place

1. **Launch system** with perception
2. **Spawn objects** in Gazebo (cylinders, boxes)
3. **Perception detects** them automatically
4. **MoveIt planning** scene updated with collision objects
5. **Plan motion** using MoveIt - robot avoids detected obstacles
6. **Grasp planning** can target detected cylinders/boxes

## References

- **Inspiration:** [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2/tree/jazzy)
- **MoveIt 2 Collision Objects:** [MoveIt 2 Tutorials](https://moveit.picknik.ai/)
- **PCL Documentation:** [Point Cloud Library](https://pointclouds.org/)
- **RANSAC Algorithm:** [Wikipedia - RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus)
- **DBSCAN Clustering:** [sklearn DBSCAN](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html)

---

🎯 **Quick Start:**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
# Wait 10 seconds
ros2 launch arm_perception perception.launch.py
# Add objects in Gazebo and watch them appear in MoveIt planning scene!
```
