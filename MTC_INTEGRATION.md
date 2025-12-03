# MoveIt Task Constructor Integration Guide

This document provides a complete guide for the MoveIt Task Constructor (MTC) integration in the LDR humanoid arm system.

## Overview

MoveIt Task Constructor has been integrated to enable advanced task-level manipulation planning. This integration provides:

- **Sequential task composition** - Build complex tasks from reusable stages
- **Pick-and-place automation** - Complete pipeline for manipulation tasks
- **Multiple planning strategies** - Cartesian, sampling-based, and interpolation planners
- **Demo applications** - Ready-to-run examples for learning and testing

## Installation

### Step 1: Build the Workspace (without MTC)

First, build the workspace to get the check tool:

```bash
cd /path/to/ldr-humanoid-arm-system

# Build the packages
colcon build --packages-select arm_moveit_config arm_demos

# Source the workspace
source install/setup.bash
```

### Step 2: Check MTC Installation Status

```bash
# Run the MTC installation checker
ros2 run arm_demos check_mtc.py
```

If MTC is **not installed**, you'll see:
```
============================================================
STATUS: MTC NOT FULLY INSTALLED
============================================================
```

### Step 3: Install MTC Dependencies

```bash
# Install MoveIt Task Constructor packages for ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-moveit-task-constructor-*

# Source ROS 2 installation to get new packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Step 4: Verify Installation

```bash
# Run the check again
ros2 run arm_demos check_mtc.py

# You should now see:
# ============================================================
# STATUS: MTC FULLY INSTALLED ✓
# ============================================================
```

Alternative verification:
```bash
# Check that MTC packages are available
ros2 pkg list | grep task_constructor

# Expected output:
# moveit_task_constructor_capabilities
# moveit_task_constructor_core
# moveit_task_constructor_msgs
# moveit_task_constructor_visualization
```

## Files Created

### Configuration Files (2)

1. **[src/planning/arm_moveit_config/config/mtc_capabilities.yaml](src/planning/arm_moveit_config/config/mtc_capabilities.yaml)**
   - MTC capabilities for move_group
   - Execution parameters (scaling, timeouts)
   - Cartesian planning configuration

2. **[src/planning/arm_moveit_config/config/mtc_solvers.yaml](src/planning/arm_moveit_config/config/mtc_solvers.yaml)**
   - Cartesian path planner settings
   - Joint interpolation configuration
   - OMPL and Pilz planner parameters

### Demo Scripts (3)

3. **[src/applications/arm_demos/scripts/check_mtc.py](src/applications/arm_demos/scripts/check_mtc.py)** ⭐ NEW
   - MTC installation checker
   - Verifies all required packages are installed
   - Provides installation instructions if missing
   - Executable: `ros2 run arm_demos check_mtc.py`

4. **[src/applications/arm_demos/scripts/mtc_simple_demo.py](src/applications/arm_demos/scripts/mtc_simple_demo.py)**
   - Basic MTC demonstration
   - Task: home → approach → retreat → home
   - No gripper required
   - Executable: `ros2 run arm_demos mtc_simple_demo.py`
   - **Requires MTC to be installed**

5. **[src/applications/arm_demos/scripts/mtc_pick_place_demo.py](src/applications/arm_demos/scripts/mtc_pick_place_demo.py)**
   - Advanced pick-and-place pipeline
   - Includes grasp generation, object attachment
   - Will be fully functional when gripper is integrated
   - Executable: `ros2 run arm_demos mtc_pick_place_demo.py`
   - **Requires MTC to be installed**

### Launch Files (1)

6. **[src/applications/arm_demos/launch/mtc_demo.launch.py](src/applications/arm_demos/launch/mtc_demo.launch.py)**
   - Launches complete MTC environment
   - Options: `demo:=simple|pick_place|none`
   - Supports Gazebo simulation
   - Configurable RViz visualization
   - **Auto-detects if MTC is installed** and warns if not

### Documentation (3)

7. **[CLAUDE.md](CLAUDE.md)** (updated)
   - Added MTC section with usage examples
   - Updated control architecture (now 3 paths)
   - Added configuration file references

8. **[src/applications/arm_demos/README_MTC.md](src/applications/arm_demos/README_MTC.md)**
   - Comprehensive MTC guide
   - Code patterns and examples
   - Debugging and troubleshooting

9. **[MTC_INTEGRATION.md](MTC_INTEGRATION.md)** (this file)
   - Complete installation guide
   - Usage instructions
   - Troubleshooting

### Package Files Updated (3)

10. **[src/planning/arm_moveit_config/package.xml](src/planning/arm_moveit_config/package.xml)**
    - Added 4 MTC dependencies

11. **[src/applications/arm_demos/package.xml](src/applications/arm_demos/package.xml)**
    - Added 2 MTC dependencies

12. **[src/applications/arm_demos/CMakeLists.txt](src/applications/arm_demos/CMakeLists.txt)**
    - Added executable installation for demo scripts

## Usage

> **⚠️ IMPORTANT:** Before running MTC demos, verify that MTC is installed:
> ```bash
> ros2 run arm_demos check_mtc.py
> ```
> If not installed, follow the [Installation](#installation) section above.

### Option 1: All-in-One Launch (Easiest)

Launch everything (Gazebo + MoveIt + MTC demo) in one command:

```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

**What this does:**
- Starts Gazebo Harmonic with robot
- Launches MoveIt move_group with MTC capabilities
- Opens RViz with visualization
- Runs the simple MTC demo script

### Option 2: Modular Launch (Recommended for Development)

Launch components separately for better debugging:

```bash
# Terminal 1: Launch Gazebo + Controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: Launch MTC demo (wait ~20 seconds after terminal 1)
ros2 launch arm_demos mtc_demo.launch.py demo:=simple
```

### Option 3: MTC Environment Only

Launch just the MTC environment without running a demo:

```bash
# With Gazebo
ros2 launch arm_demos mtc_demo.launch.py demo:=none use_sim:=true

# Without Gazebo (for real hardware)
ros2 launch arm_demos mtc_demo.launch.py demo:=none use_sim:=false
```

Then run demo scripts manually:
```bash
ros2 run arm_demos mtc_simple_demo.py
```

### Launch Arguments

All `mtc_demo.launch.py` arguments:

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `demo` | `simple`, `pick_place`, `none` | `simple` | Which demo to run |
| `use_sim` | `true`, `false` | `false` | Use Gazebo simulation |
| `use_rviz` | `true`, `false` | `true` | Launch RViz |
| `log_level` | `DEBUG`, `INFO`, `WARN`, `ERROR` | `INFO` | Logging level |

**Examples:**

```bash
# Simple demo with Gazebo and debug logging
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true log_level:=DEBUG

# Pick-and-place demo without RViz
ros2 launch arm_demos mtc_demo.launch.py demo:=pick_place use_rviz:=false

# Environment only, no simulation
ros2 launch arm_demos mtc_demo.launch.py demo:=none use_sim:=false
```

## Testing

### Test 1: Simple Demo

This demo shows basic MTC concepts without requiring a gripper.

```bash
# Launch with Gazebo
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

**Expected behavior:**
1. Gazebo opens with the arm in default position
2. RViz opens with MoveIt visualization
3. After ~8 seconds, the demo script starts
4. The arm moves to home position
5. Approaches (moves forward)
6. Retreats (moves backward)
7. Returns to home
8. Script prompts "Execute the task? (y/n)"
9. Type 'y' to execute the planned trajectory

### Test 2: Pick-and-Place Demo (Future)

This demo requires gripper integration to be fully functional.

```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=pick_place use_sim:=true
```

**Current status:**
- Task creation and planning stages work
- Gripper-related stages are commented out (TODO markers)
- Will be enabled when gripper/hand package is integrated

### Test 3: Custom Script

Run the demo scripts directly:

```bash
# First, launch Gazebo and MoveIt separately
ros2 launch arm_control sim.launch.py

# In another terminal, wait 20s then run:
ros2 run arm_demos mtc_simple_demo.py
```

## Verification Steps

### 1. Check Controllers

```bash
ros2 control list_controllers

# Expected output should include:
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### 2. Check Move Group

```bash
ros2 node list | grep move_group

# Expected: /move_group
```

### 3. Check MTC Capability

```bash
ros2 service list | grep execute_task_solution

# Expected: /move_group/execute_task_solution
```

### 4. Monitor Joint States

```bash
ros2 topic echo /joint_states --once
```

### 5. Test MTC Demo Planning

The demo script should output:
```
[INFO] [mtc_simple_demo]: MTC Simple Demo initialized
[INFO] [mtc_simple_demo]: Simple MTC task created
[INFO] [mtc_simple_demo]: Planning task...
[INFO] [mtc_simple_demo]: Found X solution(s)
[INFO] [mtc_simple_demo]: Best solution cost: Y

Execute the task? (y/n):
```

## Architecture

### Control Paths

The system now provides **three control paths**:

1. **Simple Motion Planner** ([arm_control](src/control/arm_control/))
   - Direct trajectory control
   - Fast and lightweight
   - Good for known trajectories

2. **MoveIt Planning** ([arm_moveit_config](src/planning/arm_moveit_config/))
   - Collision avoidance
   - Inverse kinematics
   - Cartesian planning

3. **MoveIt Task Constructor** (NEW)
   - Sequential task composition
   - Pick-and-place automation
   - Multi-step manipulation

All three paths use the same controller interface: `arm_controller/follow_joint_trajectory`

### MTC Pipeline

```
User Script
    ↓
MTC Task (composed of stages)
    ↓
Stage Planners (Cartesian, OMPL, Interpolation)
    ↓
MoveIt move_group (with ExecuteTaskSolutionCapability)
    ↓
arm_controller (JointTrajectoryController)
    ↓
Gazebo/Hardware
```

## Common Issues

### Issue: "No solutions found"

**Cause:** Planning constraints too tight or unreachable goal

**Solutions:**
1. Check joint limits: [src/planning/arm_moveit_config/config/joint_limits.yaml](src/planning/arm_moveit_config/config/joint_limits.yaml)
2. Increase planning timeout in stage configuration
3. Reduce velocity/acceleration scaling
4. Verify IK frame is correct: `wrist_roll_link`

### Issue: "Task execution failed"

**Cause:** Controllers not active or move_group not configured

**Solutions:**
1. Verify controllers: `ros2 control list_controllers`
2. Check ExecuteTaskSolutionCapability is loaded
3. Ensure Gazebo is fully initialized (wait 20+ seconds)
4. Review [mtc_capabilities.yaml](src/planning/arm_moveit_config/config/mtc_capabilities.yaml)

### Issue: "ImportError: cannot import name 'core'"

**Cause:** MTC Python bindings not installed

**Solution:**
```bash
sudo apt install ros-jazzy-moveit-task-constructor-core
source /opt/ros/jazzy/setup.bash
```

### Issue: Demo script hangs at planning

**Cause:** move_group not running or not responding

**Solutions:**
1. Check move_group is running: `ros2 node list | grep move_group`
2. Restart move_group: Kill and relaunch
3. Check for errors in move_group terminal output
4. Verify robot model loaded: Look for "Loading robot model" in logs

## Next Steps

### Short Term

1. **Test the simple demo**
   ```bash
   ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
   ```

2. **Review MTC documentation**
   - Read [src/applications/arm_demos/README_MTC.md](src/applications/arm_demos/README_MTC.md)
   - Check [CLAUDE.md](CLAUDE.md) MTC section

3. **Experiment with custom tasks**
   - Copy `mtc_simple_demo.py` as a template
   - Add your own stages
   - Test planning and execution

### Long Term

1. **Integrate gripper/hand**
   - Enable gripper stages in pick-and-place demo
   - Add grasp database for common objects

2. **Add perception integration**
   - Object detection for autonomous grasping
   - Dynamic obstacle avoidance

3. **Create task library**
   - Reusable manipulation primitives
   - Standard pick-and-place sequences
   - Multi-step assembly tasks

4. **Optimize performance**
   - Tune planner parameters
   - Profile planning times
   - Optimize stage configurations

## Resources

### Documentation

- [MTC Official Documentation](https://ros-planning.github.io/moveit_task_constructor/)
- [MTC Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html)
- [MoveIt 2 Documentation](https://moveit.ros.org/)
- [Local MTC Guide](src/applications/arm_demos/README_MTC.md)

### Code Examples

- Simple demo: [mtc_simple_demo.py](src/applications/arm_demos/scripts/mtc_simple_demo.py)
- Pick-place demo: [mtc_pick_place_demo.py](src/applications/arm_demos/scripts/mtc_pick_place_demo.py)
- Launch file: [mtc_demo.launch.py](src/applications/arm_demos/launch/mtc_demo.launch.py)

### Configuration

- Capabilities: [mtc_capabilities.yaml](src/planning/arm_moveit_config/config/mtc_capabilities.yaml)
- Solvers: [mtc_solvers.yaml](src/planning/arm_moveit_config/config/mtc_solvers.yaml)
- Joint limits: [joint_limits.yaml](src/planning/arm_moveit_config/config/joint_limits.yaml)

## Support

For issues, questions, or contributions:

1. Check troubleshooting section in [README_MTC.md](src/applications/arm_demos/README_MTC.md)
2. Review [CLAUDE.md](CLAUDE.md) for system architecture
3. Open an issue on the repository
4. Consult MTC official documentation

## License

MIT License - See repository LICENSE file

---

**Integration completed:** 2024-11-19
**ROS 2 Version:** Jazzy
**MoveIt Version:** 2.x
**MTC Version:** Latest (Jazzy)
