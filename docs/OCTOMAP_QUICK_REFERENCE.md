# OctoMap Quick Reference

Quick commands for running and verifying OctoMap setup.

## Quick Start (Single Command)

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash  # For native displays
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

**Timeline:**
- T=0s: Gazebo + Robot spawns
- T=10s: OctoMap server starts
- T=12s: RViz opens with all displays configured

## Interactive Demo Script

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
./run_octomap_demo.sh
```

Choose from 3 options:
1. Integrated launch (all-in-one)
2. Manual launch (multiple terminals)
3. Just RViz (if system already running)

## Manual Launch (3 Terminals)

### Terminal 1: Main System
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

### Terminal 2: OctoMap Server (wait 25s)
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
ros2 launch arm_system_bringup octomap_server.launch.py use_sim_time:=true
```

### Terminal 3: RViz (wait 30s)
```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
source install/setup.bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash  # Optional: for native displays
rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz
```

## Verification Commands

### Check OctoMap is Running
```bash
ros2 node list | grep octomap
# Should show: /octomap_server
```

### Check Topics
```bash
# List all octomap topics
ros2 topic list | grep octomap

# Should show:
# /octomap_binary
# /octomap_full
# /octomap_point_cloud_centers
# /projected_map
```

### Check Map is Publishing
```bash
# Check rate (should be ~1 Hz)
ros2 topic hz /octomap_binary

# Check map size (should grow over time)
ros2 topic echo /octomap_binary --field data --once | wc -c
# Empty: ~45 bytes
# Populated: 1000+ bytes
```

### Check Camera Input
```bash
# Should be ~30 Hz
ros2 topic hz /camera/depth/points
```

### Check MoveIt Integration
```bash
# Verify octomap frame
ros2 param get /move_group octomap_frame
# Should show: map
```

## RViz Display Configuration

### Already Configured (Default)
The custom RViz config includes:
- Grid
- MotionPlanning
- Camera_PointCloud (`/camera/depth/points`)
- OctoMap_Voxels (`/octomap_point_cloud_centers`, red squares)

### Add Native OccupancyMap Display

**Requirements:** Overlay workspace must be sourced

**Steps:**
1. In RViz, click **Add** button
2. Go to **By topic** tab
3. Expand `/octomap_binary`
4. Select **OccupancyMap**
5. Click **OK**

**Configure:**
- Voxel Rendering: Occupied voxels
- Voxel Coloring: Z-Axis (colorful height gradient)
- Alpha: 1.0

### Add OccupancyGrid Display (2D Slice)

**Steps:**
1. In RViz, click **Add** button
2. Go to **By topic** tab
3. Expand `/octomap_binary`
4. Select **OccupancyGrid**
5. Click **OK**

**Configure:**
- Slice Height: 0.5 (meters above ground)
- Color Scheme: Map
- Alpha: 0.7

## Save Current Map

```bash
cd ~/ros2_ws/ldr-humanoid-arm-system
python3 save_octomap.py /tmp/current_map.bt
```

## View Map in octovis

```bash
# Save map first
python3 save_octomap.py /tmp/current_map.bt

# Open in native viewer (colorful 3D voxels)
octovis /tmp/current_map.bt
```

## Troubleshooting

### RViz shows no voxels

**Check 1:** Fixed Frame
- Should be `map` (not `base_link`)

**Check 2:** Point size
- Increase to 0.1m in display settings

**Check 3:** Map is populated
```bash
ros2 topic echo /octomap_binary --field data --once | wc -c
# Should be > 1000 bytes
```

### "undefined symbol" error

**Solution:** Source the overlay workspace
```bash
source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
```

### OctoMap warnings at startup

**Normal warnings (can ignore):**
- "Nothing to publish, octree is empty" - Map builds over 10-15s
- "Could not open file" - No pre-existing map file (expected)

### Camera not publishing

```bash
# Check camera topic
ros2 topic hz /camera/depth/points

# If nothing, restart Gazebo
```

## Key Configuration Files

| File | Purpose |
|------|---------|
| [octomap_server.launch.py](src/bringup/arm_system_bringup/launch/octomap_server.launch.py) | Launch OctoMap node |
| [octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml) | OctoMap parameters |
| [moveit_with_octomap.rviz](src/planning/arm_moveit_config/config/moveit_with_octomap.rviz) | RViz configuration |
| [moveit_gazebo_with_octomap.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py) | Integrated launch |
| [sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml) | MoveIt sensor config |

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| resolution | 0.05 | Voxel size (5cm) |
| frame_id | map | Fixed frame |
| base_frame_id | base_link | Robot frame |
| max_range | 5.0 | Sensor max range (meters) |
| sensor_model/hit | 0.7 | Occupied probability |
| sensor_model/miss | 0.4 | Free probability |

## Documentation Links

- [OCTOMAP_QUICKSTART.md](OCTOMAP_QUICKSTART.md) - Detailed step-by-step guide
- [OCTOMAP_INTEGRATED_LAUNCH.md](OCTOMAP_INTEGRATED_LAUNCH.md) - Integrated launch details
- [OCTOMAP_RVIZ_NATIVE_DISPLAYS.md](OCTOMAP_RVIZ_NATIVE_DISPLAYS.md) - Native display usage
- [OCTOMAP_SETUP_COMPLETE.md](OCTOMAP_SETUP_COMPLETE.md) - Complete overview

## Scripts

| Script | Purpose |
|--------|---------|
| [run_octomap_demo.sh](run_octomap_demo.sh) | Interactive demo launcher |
| [rebuild_octomap_plugin.sh](rebuild_octomap_plugin.sh) | Rebuild RViz plugin |
| [save_octomap.py](save_octomap.py) | Save map to file |

---

**Ready to run:** Everything is configured and ready to use!
