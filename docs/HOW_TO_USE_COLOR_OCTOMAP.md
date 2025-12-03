# How to Use Color OctoMap - Step by Step

Quick guide to launch and view the colorful OctoMap voxels.

## Quick Start (Single Command)

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

Wait ~15 seconds and you'll see:
- Gazebo with the robot
- RViz with colorful voxel cubes building up automatically

## What You'll See in RViz

### Displays Panel (Left Side)

You should see these displays checked/enabled:
- ✅ **Grid** - Ground reference
- ✅ **MotionPlanning** - MoveIt interface with robot
- ✅ **Camera_PointCloud** - Colorful raw camera data
- ✅ **OctoMap_Color_Voxels** - **Colorful cube voxels** (this is what you want!)

### 3D View (Right Side)

You'll see:
- **Robot arm** (orange/gray)
- **Interactive markers** (blue/green spheres for planning)
- **Colorful point cloud** (dense, from camera)
- **Colorful cube voxels** (purple, orange, green, etc.) - These represent the 3D map!

## Step-by-Step Manual Launch (3 Terminals)

If you prefer to launch components separately:

### Terminal 1: Launch Gazebo + MoveIt
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

**Wait ~20 seconds** for Gazebo to fully load.

### Terminal 2: Launch Color OctoMap Server
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
ros2 launch arm_system_bringup octomap_server.launch.py \
  use_sim_time:=true \
  use_color_octomap:=true
```

You should see:
```
[color_octomap_server_node]: Subscribing to /camera/depth/points
```

### Terminal 3: Launch RViz with OctoMap Config
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz
```

## Verify Everything is Working

### Check Topics are Publishing

```bash
# Check color voxels are published (should be ~1 Hz)
ros2 topic hz /occupied_cells_vis_array

# Check camera is working (should be ~30 Hz)
ros2 topic hz /camera/depth/points

# Check octomap binary is published (should be ~1 Hz)
ros2 topic hz /octomap_binary
```

### Check Number of Voxels

```bash
# See how many colored cubes are in the map
ros2 topic echo /occupied_cells_vis_array --field markers --once | wc -l
```

This number should **grow over time** as the map builds. Start small (~10-50) and grow to hundreds/thousands.

## In RViz: Enable/Disable Displays

### To See ONLY Colored Voxels

In the **Displays** panel (left side):
1. ✅ Keep **OctoMap_Color_Voxels** checked
2. ❌ Uncheck **Camera_PointCloud** (to hide raw camera data)
3. ✅ Keep **Grid** and **MotionPlanning** checked

### To Adjust Voxel Display

Click on **OctoMap_Color_Voxels** in the Displays panel to expand options:
- **Topic:** Should be `/occupied_cells_vis_array`
- **Value:** Should be checked (✅)

If you don't see voxels, check the **Status** field - it should say "OK" with a number of markers.

## Common Issues

### I don't see any colored voxels

**Solution 1:** Wait 15-20 seconds for map to build up

**Solution 2:** Check Fixed Frame
- Global Options → Fixed Frame
- Should be: `base_fixture_link`

**Solution 3:** Verify topic is publishing
```bash
ros2 topic echo /occupied_cells_vis_array --field markers --once
# Should show array of markers
```

### I only see one color / gray voxels

**Solution 1: Check colored_map parameter**
```bash
ros2 param get /octomap_server colored_map
# Should show: Boolean value is: True
```

**If it shows False:**
The `colored_map` parameter must be set to `true` in the config file. This is already fixed in [octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml:34), so just rebuild and restart:
```bash
colcon build --packages-select arm_system_bringup
source install/setup.bash
# Restart the system
```

**Solution 2: Check you're using color octomap node:**
```bash
ros2 node info /octomap_server
# Should show: /color_octomap_server_node (not octomap_server_node)
```

**If it shows octomap_server_node (grayscale):**
Restart with color enabled:
```bash
ros2 launch arm_system_bringup octomap_server.launch.py \
  use_sim_time:=true \
  use_color_octomap:=true
```

### Voxels are too small / too big

**Increase voxel resolution** (smaller voxels):
```bash
ros2 launch arm_system_bringup octomap_server.launch.py \
  use_sim_time:=true \
  use_color_octomap:=true \
  resolution:=0.03  # 3cm voxels instead of 5cm
```

**Decrease resolution** (bigger voxels):
```bash
resolution:=0.1  # 10cm voxels
```

## RViz Camera Controls

To get a better view of the colorful voxels:

**Mouse controls:**
- **Left click + drag:** Rotate view
- **Middle click + drag:** Pan view
- **Scroll wheel:** Zoom in/out
- **Shift + left click:** Move view to clicked point

**Recommended view:**
- Orbit around the robot to see the 3D voxels from different angles
- Zoom in to see individual colored cubes
- The voxels should look like LEGO blocks in different colors!

## Saving the Colorful Map

To save and view the map later:

```bash
# While system is running
python3 save_octomap.py /tmp/colorful_map.bt

# View in octovis (even more colorful!)
octovis /tmp/colorful_map.bt
```

The octovis viewer shows the voxels with even better graphics and lighting.

## Quick Reference

| What | Command |
|------|---------|
| Launch everything | `ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py` |
| Check voxels publishing | `ros2 topic hz /occupied_cells_vis_array` |
| Count voxels | `ros2 topic echo /occupied_cells_vis_array --field markers --once \| wc -l` |
| Save map | `python3 save_octomap.py /tmp/map.bt` |
| View saved map | `octovis /tmp/map.bt` |

## What the Colors Mean

The colors come directly from the RGB camera:
- **Purple/Magenta:** Objects that appear purple in the camera view
- **Orange:** Objects that appear orange (like robot parts)
- **Green:** Green objects or lighting
- **Blue:** Blue objects
- **Yellow/Cyan:** Other colors from the environment

The voxels show the **actual colors** of objects in the environment as seen by the depth camera!

## Summary

**To use color OctoMap:**
1. Launch: `ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py`
2. Wait 15-20 seconds
3. Look for **OctoMap_Color_Voxels** display in RViz
4. You should see colorful cube voxels building up!

**The voxels will grow and accumulate as the camera observes the environment.**
