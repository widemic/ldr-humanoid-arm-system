# Quick Start: Colorful OctoMap Voxels

## Simple Usage

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash

# Launch everything with colorful voxels
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

Wait 15-20 seconds for everything to initialize. You'll see:
- Gazebo simulation with robot
- RViz with colorful 3D voxel cubes
- 10 vibrant colors (purple, orange, cyan, green, gold, pink, teal, violet, etc.)

## What's Happening Automatically

1. **t=0s**: Gazebo + robot spawns
2. **t=10s**: OctoMap server starts (listening to camera)
3. **t=11s**: Colorizer node starts (making voxels colorful)
4. **t=12s**: RViz opens showing colored voxels

## Expected Result in RViz

In the **Displays** panel you'll see:
- ✅ **Camera_PointCloud** - Raw camera data (small colored points)
- ✅ **OctoMap_Color_Voxels** - Colorful 3D cubes (MarkerArray display)

The voxels will be **colorful cube blocks** like LEGO pieces in 10 different vibrant colors!

## Topics Published

- `/occupied_cells_vis_array` - Grayscale markers (from octomap_server)
- `/occupied_cells_vis_array_colored` - **Colorful markers (from colorizer node)** ← RViz uses this!
- `/octomap_point_cloud_centers` - Point cloud version of voxels
- `/camera/depth/points` - Raw camera point cloud with RGB data

## Verification Commands

```bash
# Check colorizer is running
ros2 node list | grep colorize
# Should show: /colorize_octomap_markers

# Check colorful markers are publishing
ros2 topic hz /occupied_cells_vis_array_colored
# Should show: ~1 Hz (updates when map changes)

# See the vibrant colors in markers
ros2 topic echo /occupied_cells_vis_array_colored --field markers[0].color --once
# Should show: r: 0.541, g: 0.169, b: 0.886, a: 1.0 (purple!)
```

## Troubleshooting

### I don't see any voxels

**Wait longer** - It takes time for the map to build:
1. Move the robot arm (use MoveIt interactive markers in RViz)
2. Wait 10-20 seconds for voxels to appear
3. Voxels only appear where the camera sees obstacles

### Voxels are gray/white instead of colorful

**Check the topic:**
1. In RViz → **Displays** → **MarkerArray** → **OctoMap_Color_Voxels**
2. Expand **Topic**
3. Verify it says: `/occupied_cells_vis_array_colored` (with "_colored")
4. If it says `/occupied_cells_vis_array` (without "_colored"), change it

### Colorizer not running

```bash
# Check if it's running
ros2 node list | grep colorize

# If not running, start manually
cd ~/ros2_ws/ldr-humanoid-arm-system
python3 colorize_octomap_markers.py
```

### System crashes (segmentation fault)

This is usually unrelated to the colorizer. Try:
1. Close all terminals
2. Source workspace: `source install/setup.bash`
3. Launch again: `ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py`

## Manual Color Change in RViz

If you want a **single color** instead of random colors:

1. In RViz → **Displays** → **MarkerArray**
2. Expand **Namespaces**
3. Click the **colored box** next to "map"
4. Choose your color (purple, orange, green, etc.)

This overrides all voxels to one color.

## Files Involved

- **Colorizer Node**: `colorize_octomap_markers.py` (workspace root)
- **Launch File**: `src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py`
- **RViz Config**: `src/planning/arm_moveit_config/config/moveit_with_octomap.rviz`
- **OctoMap Config**: `src/bringup/arm_system_bringup/config/octomap_server.yaml`

## Summary

🎨 **One command to get colorful 3D voxels:**

```bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

Enjoy your colorful LEGO-block style 3D map! 🟣🟠🟢��🔵
