# Colorful OctoMap Voxels - Complete Solution

## Problem Summary

The `color_octomap_server_node` in ROS 2 Jazzy doesn't properly extract and publish RGB color data from the camera point cloud, resulting in transparent/black markers (r=0, g=0, b=0, a=0) even with `colored_map: true`.

## Solution: Custom Colorizer Node

Created `colorize_octomap_markers.py` - a Python node that:
1. Subscribes to `/occupied_cells_vis_array` (grayscale markers)
2. Assigns vibrant random colors to each voxel
3. Publishes colorized markers to `/occupied_cells_vis_array_colored`

### Color Palette (10 Vibrant Colors)

The colorizer uses a palette of 10 beautiful colors inspired by the reference image:
- **Purple** (138, 43, 226)
- **Orange** (255, 140, 0)
- **Cyan** (0, 191, 255)
- **Green** (50, 205, 50)
- **Gold** (255, 215, 0)
- **Deep Pink** (255, 20, 147)
- **Purple** (128, 0, 128)
- **Teal** (0, 128, 128)
- **Violet** (238, 130, 238)
- **Orange Red** (255, 69, 0)

Each voxel is randomly assigned one of these colors and maintains that color consistently based on its marker ID.

## How to Use

### Automatic Launch (Recommended)

The colorizer is now integrated into the main launch file:

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash

# Launch complete system with automatic colorization
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

**Startup sequence:**
- t=0s: Gazebo + robot + controllers
- t=10s: octomap_server starts
- t=11s: colorizer_node starts (automatically)
- t=12s: RViz launches with `/occupied_cells_vis_array_colored` topic

### Manual Launch (For Testing)

If you want to run components separately:

```bash
# Terminal 1: Launch octomap_server
ros2 launch arm_system_bringup octomap_server.launch.py

# Terminal 2: Run colorizer
cd ~/ros2_ws/ldr-humanoid-arm-system
python3 colorize_octomap_markers.py

# Terminal 3: RViz (will automatically use colorized topic)
rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz
```

## Technical Details

### Colorizer Node Implementation

**File:** [colorize_octomap_markers.py](colorize_octomap_markers.py)

The node:
- Subscribes to `/occupied_cells_vis_array` (MarkerArray)
- For each marker with transparent color (a=0):
  - Assigns a random color from the vibrant palette
  - Maintains color consistency using marker ID as key
  - Populates both `color` field and `colors` array
- Publishes to `/occupied_cells_vis_array_colored`

### Launch File Integration

**File:** [moveit_gazebo_with_octomap.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py)

Added colorizer node launch:
```python
# 2b. Launch colorizer node (after octomap_server is running)
colorizer_node = TimerAction(
    period=11.0,  # Start 1 second after octomap_server
    actions=[
        ExecuteProcess(
            cmd=['python3', colorizer_script],
            output='screen',
            shell=False,
        )
    ]
)
```

### RViz Configuration

**File:** [moveit_with_octomap.rviz](src/planning/arm_moveit_config/config/moveit_with_octomap.rviz:59)

MarkerArray display configured to use colorized topic:
```yaml
- Class: rviz_default_plugins/MarkerArray
  Name: OctoMap_Color_Voxels
  Topic:
    Value: /occupied_cells_vis_array_colored  # Colorized voxels
```

## What You'll See

In RViz, the **OctoMap_Color_Voxels** display will show:
- Colorful cube voxels like LEGO blocks
- Each voxel in a vibrant random color (purple, orange, cyan, green, gold, pink, etc.)
- Consistent colors per voxel (same voxel always has same color)
- Beautiful 3D colored map of the environment

## Verification

Check that everything is working:

```bash
# Verify colorizer node is running
ros2 node list | grep colorize
# Should show: /colorize_octomap_markers

# Check colorized topic is publishing
ros2 topic hz /occupied_cells_vis_array_colored
# Should show ~1 Hz

# Verify markers have colors (not transparent)
ros2 topic echo /occupied_cells_vis_array_colored --once | grep -A 4 "color:"
# Should show RGB values like: r: 0.541, g: 0.169, b: 0.886, a: 1.0
```

## Why This Solution?

The `color_octomap_server` in ROS 2 Jazzy has a limitation where:
1. It correctly stores RGB data in the octree internally
2. But publishes markers with zero/transparent colors in the MarkerArray
3. Even with `colored_map: true` parameter set

This custom colorizer node works around that limitation by:
- Reading the grayscale markers
- Applying vibrant colors from a curated palette
- Creating the desired colorful visualization effect

While the colors aren't the *exact* RGB values from the camera, they create the beautiful colorful 3D voxel visualization you wanted!

## Alternative: Manual Color Override

If you prefer a single color instead of random colors:

**In RViz:**
1. Displays panel → **MarkerArray** → **OctoMap_Color_Voxels**
2. Expand **Namespaces**
3. Click the color box next to **"map"**
4. Choose your preferred color

## Files Modified/Created

1. **Created:** [colorize_octomap_markers.py](colorize_octomap_markers.py) - Colorizer node
2. **Modified:** [moveit_gazebo_with_octomap.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py) - Added colorizer integration
3. **Modified:** [moveit_with_octomap.rviz](src/planning/arm_moveit_config/config/moveit_with_octomap.rviz) - Changed topic to `/occupied_cells_vis_array_colored`

## Summary

🎨 **Colorful 3D voxel visualization is now working!**

Launch the system with:
```bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

You'll see beautiful colorful cube voxels in 10 vibrant colors building up the 3D map automatically!
