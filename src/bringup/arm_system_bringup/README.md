# Arm System Bringup

Top-level launch files for the humanoid arm system. These launch files properly orchestrate all necessary nodes in the correct order to avoid duplicate services and ensure clean startup.

## Quick Start

**For full simulation with Gazebo, MoveIt, and RViz (recommended):**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

This single command launches:
- Gazebo Harmonic simulation with the robot
- MoveIt motion planning server
- RViz with MoveIt Motion Planning plugin for interactive planning

## Launch Files

### 1. Gazebo + MoveIt + RViz (`moveit_gazebo.launch.py`) ⭐ **RECOMMENDED**

**Complete integrated simulation environment** - Launches Gazebo simulation, MoveIt motion planning, and RViz visualization all together.

**Usage:**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

**Arguments:**
- `use_rviz` (default: `true`) - Start RViz2 with MoveIt Motion Planning plugin
- `world` (default: arm_gazebo lab.sdf) - Path to Gazebo world file
- `log_level` (default: `info`) - Logging level (debug, info, warn, error)

**Example without RViz:**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py use_rviz:=false
```

**Example with custom world:**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py world:=/path/to/world.sdf
```

**What it launches (in sequence):**
1. **T=0s**: Gazebo Harmonic with specified world, robot spawned, clock bridge, robot_state_publisher
2. **T=3s**: Joint State Broadcaster spawner
3. **T=4s**: Arm Controller spawner (joint_trajectory_controller)
4. **T=6s**: MoveGroup node (motion planning server)
5. **T=8s**: RViz2 with MoveIt Motion Planning plugin

The delays ensure proper initialization order and avoid timing issues.

**How to use:**
- Use the **MotionPlanning** panel in RViz to plan and execute motions
- Drag the interactive markers to set goal poses
- Click "Plan" to compute a trajectory
- Click "Execute" to run the planned trajectory in Gazebo

### 2. MoveIt Demo Mode (`moveit_demo.launch.py`)

Launches MoveIt with mock hardware (FakeSystem) for motion planning and visualization without physical hardware or simulation.

**Usage:**
```bash
ros2 launch arm_system_bringup moveit_demo.launch.py
```

**Arguments:**
- `use_rviz` (default: `true`) - Start RViz2 for visualization
- `rviz_config` (default: arm_moveit_config moveit.rviz) - Path to RViz config file

**Example with custom RViz config:**
```bash
ros2 launch arm_system_bringup moveit_demo.launch.py use_rviz:=true rviz_config:=/path/to/config.rviz
```

**Example without RViz:**
```bash
ros2 launch arm_system_bringup moveit_demo.launch.py use_rviz:=false
```

**What it launches:**
1. Static TF publisher (world → base_link)
2. Robot State Publisher
3. ros2_control node (controller_manager) with mock hardware
4. Joint State Broadcaster
5. Arm Controller (joint_trajectory_controller)
6. MoveGroup node (motion planning server)
7. RViz2 (optional)


## Architecture

These launch files are designed to:
- **Avoid duplicate services**: Each service is started only once in the correct order
- **Proper sequencing**: Nodes are started in dependency order (e.g., robot_state_publisher before controllers)
- **Clean separation**: Demo mode vs Gazebo mode are separate launch files with appropriate configurations
- **Configurable**: Launch arguments allow customization without editing files

## Comparison with arm_moveit_config/demo.launch.py

The original `demo.launch.py` uses `generate_demo_launch()` utility which automatically configures everything but provides less control. These bringup launch files:

✅ **Explicit node ordering** - You can see exactly what starts and in what order
✅ **No duplicate services** - Each node is explicitly defined once
✅ **Better debugging** - Easier to modify individual nodes or add custom parameters
✅ **More flexible** - Easy to add custom nodes or modify behavior

## Troubleshooting

### Controllers fail to spawn
- Make sure ros2_control_node is fully initialized before spawning controllers
- Check that the robot_description topic has been published

### MoveIt can't find robot
- Verify robot_state_publisher is running: `ros2 node list | grep robot_state`
- Check robot_description topic: `ros2 topic echo /robot_description --once`

### RViz crashes on startup
- This may be due to system library conflicts (especially with snap packages)
- Try running without RViz: `use_rviz:=false`
- You can launch RViz separately after the system is up

## Development

To add more launch configurations:

1. Create a new launch file in `src/bringup/arm_system_bringup/launch/`
2. Follow the same pattern of using `OpaqueFunction` for parameter access
3. Use `DeclareLaunchArgument` to make configurations flexible
4. Build the package: `colcon build --packages-select arm_system_bringup --symlink-install`
5. Source and test: `. install/setup.bash && ros2 launch arm_system_bringup your_launch.py`
