# OctoMap Color Voxel Visualization

## What Was Changed

To enable colorful voxel visualization like in the reference image, the system now uses **color_octomap_server** which stores RGB data from the camera in the octree.

## Changes Made

### 1. Enabled Color OctoMap Server

**File:** [moveit_gazebo_with_octomap.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py:79)

Added parameter:
```python
"use_color_octomap": "true",
```

This launches `color_octomap_server_node` instead of the grayscale `octomap_server_node`.

### 2. Enabled Colored Map Visualization

**File:** [octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml)

Added critical parameter:
```yaml
colored_map: true           # Enable colored voxel visualization
```

**IMPORTANT:** Even when using `color_octomap_server_node`, the `colored_map` parameter must be set to `true` for the MarkerArray to publish individual RGB colors. Without this, all voxels will display in a single blue color.

### 3. Updated RViz Configuration

**File:** [moveit_with_octomap.rviz](src/planning/arm_moveit_config/config/moveit_with_octomap.rviz)

Added **MarkerArray** display for colored voxels:

```yaml
- Class: rviz_default_plugins/MarkerArray
  Name: OctoMap_Color_Voxels
  Topic:
    Value: /occupied_cells_vis_array
  Value: true
```

Disabled the old red PointCloud2 display (set `Value: false`).

## How It Works

### Color OctoMap Server

**Node:** `color_octomap_server_node`

**What it does:**
- Subscribes to `/camera/depth/points` (RGB-D point cloud with color data)
- Extracts both **geometry** (3D position) and **color** (RGB values) from each point
- Stores colored voxels in the octree structure
- Publishes visualization as MarkerArray with cube markers

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/octomap_binary` | Octomap | Binary octree (for MoveIt) |
| `/octomap_full` | Octomap | Full octree with all data |
| `/occupied_cells_vis_array` | MarkerArray | **Colored cube markers** |
| `/octomap_point_cloud_centers` | PointCloud2 | Voxel centers (grayscale) |

## RViz Display Configuration

### Current Displays

1. **Grid** - Reference grid
2. **MotionPlanning** - MoveIt interface
3. **Camera_PointCloud** - Raw camera data (colorful points)
   - Topic: `/camera/depth/points`
4. **OctoMap_Color_Voxels** - **Colored voxel cubes** ✨
   - Topic: `/occupied_cells_vis_array`
   - Type: MarkerArray
   - Each cube has the RGB color from the camera

### Why MarkerArray Shows Colors

The `color_octomap_server` publishes each occupied voxel as a **Marker** with:
- **Type:** CUBE
- **Scale:** 0.05 × 0.05 × 0.05 meters (voxel size)
- **Color:** RGB values from the camera point cloud
- **Position:** Center of the voxel

RViz displays these cubes with their original colors, creating the colorful 3D voxel visualization!

## Running the System

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash

# Launch with color octomap enabled
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

## What You Should See

In RViz, the **OctoMap_Color_Voxels** display will show:
- **Colorful cube voxels** matching the colors from the depth camera
- Each voxel represents a 5cm × 5cm × 5cm occupied space
- Colors reflect the actual RGB data captured by the camera
- Purple, orange, blue, green, yellow voxels (like in your reference image!)

## Frame Configuration

All frames are set to `base_fixture_link`:
- **frame_id:** `base_fixture_link` (fixed frame for octree)
- **base_frame_id:** `base_fixture_link` (robot base)

The octree is built in the robot's coordinate frame, making it move-relative.

## Comparison: Grayscale vs Color

| Feature | Grayscale (old) | Color (new) |
|---------|----------------|-------------|
| Node | octomap_server_node | color_octomap_server_node |
| Data stored | Occupancy only | Occupancy + RGB |
| Visualization | Red squares | Colorful cubes |
| Topic | /octomap_point_cloud_centers | /occupied_cells_vis_array |
| Display type | PointCloud2 | MarkerArray |
| Memory usage | Lower | Higher |
| Visual appeal | Basic | Stunning! |

## Verification

Check that color octomap is running:

```bash
# Check node is running
ros2 node list | grep octomap
# Should show: /octomap_server

# Check MarkerArray is publishing
ros2 topic hz /occupied_cells_vis_array
# Should show ~1 Hz

# Check number of markers
ros2 topic echo /occupied_cells_vis_array --field markers --once | wc -l
# Should grow as map fills
```

## Performance Notes

**Color octomap uses more resources:**
- Stores RGB data for each voxel (12 extra bytes per voxel)
- MarkerArray with many cubes is heavier to render than PointCloud2
- Recommended for visualization and demonstration
- For production/navigation, grayscale octomap is more efficient

**To switch back to grayscale:**
```bash
ros2 launch arm_system_bringup octomap_server.launch.py \
  use_color_octomap:=false \
  use_sim_time:=true
```

## Summary

✅ **Color octomap server enabled**
✅ **MarkerArray display configured for colored voxels**
✅ **RViz shows colorful 3D cubes like in the reference image**
✅ **System ready to visualize environment in full color!**

Your OctoMap now has the beautiful colorful voxel visualization you wanted!
