# How to Run OctoMap with RViz

This guide shows the simplest way to run the complete OctoMap system with native displays.

## Prerequisites

The overlay workspace with the rebuilt plugin must exist:
```bash
ls ~/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib/liboctomap_rviz_plugins.so
```

If missing, run: `./rebuild_octomap_plugin.sh`

## Quick Start (Recommended)

### Single Command - Everything Automatic

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

**What happens:**
- T=0s: Gazebo + Robot + Controllers start
- T=10s: OctoMap server starts
- T=12s: RViz opens with OctoMap displays

The launch file **automatically** loads the rebuilt plugin - no extra steps needed!

## Alternative: Manual RViz Launch

If you want to launch RViz separately after the system is running:

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
./rviz_with_octomap.sh
```

This script:
- Sources both workspaces
- Sets library paths correctly
- Launches RViz with the OctoMap configuration

## What You'll See in RViz

### Pre-configured Displays

These are already in the RViz config:

1. **Grid** - Reference grid
2. **MotionPlanning** - MoveIt planning interface
3. **Camera_PointCloud** - Colorful point cloud from depth camera
   - Topic: `/camera/depth/points`
   - Color: RGB8 (shows actual camera colors)
4. **OctoMap_Voxels** - Red squares showing occupied space
   - Topic: `/octomap_point_cloud_centers`
   - Style: Squares, Size: 0.05m

### Native Displays You Can Add

With the rebuilt plugin, you can now add:

#### OccupancyMap (3D Colorful Voxels)

1. Click **Add** button in RViz
2. Go to **By topic** tab
3. Expand `/octomap_binary`
4. Select **OccupancyMap**
5. Click **OK**

**Configuration:**
- Voxel Rendering: **Occupied voxels**
- Voxel Coloring: **Z-Axis** (colorful height gradient)
- Alpha: **1.0**
- Tree Depth: **16** (full detail)

This gives you the colorful 3D voxel cubes!

#### OccupancyGrid (2D Slice)

1. Click **Add** button
2. Go to **By topic** tab
3. Expand `/octomap_binary`
4. Select **OccupancyGrid**
5. Click **OK**

**Configuration:**
- Slice Height: **0.5** (50cm above ground)
- Color Scheme: **Map**
- Alpha: **0.7**

## Verification

### Check Everything is Running

```bash
# Nodes
ros2 node list | grep -E "(gazebo|octomap|move_group)"

# Should show:
# /gazebo
# /octomap_server
# /move_group
```

### Check OctoMap is Publishing

```bash
# Map update rate (should be ~1 Hz)
ros2 topic hz /octomap_binary

# Map size (should grow from ~45 to 1000+ bytes)
ros2 topic echo /octomap_binary --field data --once | wc -c
```

### Check Camera Input

```bash
# Camera rate (should be ~30 Hz)
ros2 topic hz /camera/depth/points
```

## Troubleshooting

### Still Getting "undefined symbol" Error

This means the library path isn't set correctly.

**Solution 1:** Use the wrapper script
```bash
./rviz_with_octomap.sh
```

**Solution 2:** Use the integrated launch
```bash
# Make sure to source overlay before launching
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

The launch file will automatically set the library path for RViz.

### RViz Shows No Voxels

**Check Fixed Frame:**
- Must be set to `map` (not `base_link`)
- Global Options → Fixed Frame → map

**Check Point Size:**
- Increase to 0.1m if voxels are too small
- Display Settings → Size (m) → 0.1

**Check Map Has Data:**
```bash
ros2 topic echo /octomap_binary --field data --once | wc -c
# Should be > 1000 if populated
```

Wait 15-20 seconds after launch for map to accumulate data.

### Map is Empty

**Check camera is publishing:**
```bash
ros2 topic hz /camera/depth/points
```

If nothing, check Gazebo is running and robot has spawned.

## Files and Scripts

| File | Purpose |
|------|---------|
| [rviz_with_octomap.sh](rviz_with_octomap.sh) | Launch RViz with correct plugin |
| [run_octomap_demo.sh](run_octomap_demo.sh) | Interactive demo with options |
| [moveit_gazebo_with_octomap.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py) | Integrated launch |
| [moveit_with_octomap.rviz](src/planning/arm_moveit_config/config/moveit_with_octomap.rviz) | RViz configuration |

## Summary

**Easiest way to run:**

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

Then in RViz:
- **Add → By topic → /octomap_binary → OccupancyMap**
- Set Voxel Coloring to **Z-Axis** for colorful display

**You now have colorful 3D voxel visualization working!**
