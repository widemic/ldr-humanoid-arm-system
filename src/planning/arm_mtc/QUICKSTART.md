# Quick Start Guide - arm_mtc Package

This guide will get you up and running with MoveIt Task Constructor for the 5-DOF humanoid arm in under 5 minutes.

## Installation

### 1. Install MTC Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-moveit-task-constructor-core \
                 ros-jazzy-moveit-task-constructor-msgs \
                 ros-jazzy-moveit-task-constructor-visualization
```

### 2. Build the Package

```bash
cd ~/Documents/GitHub/ldr-humanoid-arm-system  # Or your workspace root
colcon build --packages-select arm_mtc
source install/setup.bash
```

## Running the Demo

### Option 1: Simple Move Demo (Recommended for First Time)

This demonstrates basic MTC usage with simple movements.

**Terminal 1 - Gazebo:**
```bash
source install/setup.bash
ros2 launch arm_control sim.launch.py
```

**Terminal 2 - MoveIt (wait ~20 seconds):**
```bash
source install/setup.bash
ros2 launch arm_moveit_config demo.launch.py
```

**Terminal 3 - Run Simple Demo (wait until RViz shows the robot):**
```bash
source install/setup.bash
ros2 run arm_mtc simple_move_demo.py
```

You should see the arm move to home position and back!

### Option 2: Pick and Place Demo

This demonstrates a more complex task with multiple stages.

**Terminal 1 - Gazebo:**
```bash
source install/setup.bash
ros2 launch arm_control sim.launch.py
```

**Terminal 2 - MoveIt (wait ~20 seconds):**
```bash
source install/setup.bash
ros2 launch arm_moveit_config demo.launch.py
```

**Terminal 3 - Run Pick and Place Demo:**
```bash
source install/setup.bash
ros2 launch arm_mtc mtc_demo.launch.py
```

## Visualizing Tasks in RViz

MTC provides visualization tools to inspect planned tasks:

1. In RViz (Terminal 2), click **Panels → Add New Panel**
2. Select **Motion Planning Tasks**
3. In the new panel:
   - Select the task from the dropdown
   - Browse through different solution trajectories
   - Inspect individual stages
   - Play/pause the trajectory preview

## Troubleshooting

### "Planning failed" Error

**Check controllers are running:**
```bash
ros2 control list_controllers
```

Should show:
- `arm_controller` [active]
- `joint_state_broadcaster` [active]

**Check MoveIt is ready:**
```bash
ros2 topic list | grep move_group
```

Should show several `/move_group/*` topics.

### "No solutions found"

- Increase planning timeout in [config/mtc_config.yaml](config/mtc_config.yaml)
- Verify the "home" pose exists in the SRDF
- Check joint limits aren't violated

### Import Errors

If you see Python import errors:

```bash
# Re-source after building
source install/setup.bash

# Verify package is installed
ros2 pkg list | grep arm_mtc
```

## Next Steps

1. **Modify the simple demo** - Edit [scripts/simple_move_demo.py](scripts/simple_move_demo.py)
   - Add custom joint positions
   - Create new movement sequences

2. **Explore configuration** - Edit [config/mtc_config.yaml](config/mtc_config.yaml)
   - Adjust planning parameters
   - Modify workspace bounds
   - Change approach/retreat behaviors

3. **Create custom tasks** - See [README.md](README.md) for examples
   - Learn about different stage types
   - Combine stages for complex behaviors

4. **Add gripper integration** - When gripper hardware is available
   - Uncomment gripper stages in pick_place_demo.py
   - Configure gripper controllers

## Useful Commands

```bash
# List all MTC-related packages
ros2 pkg list | grep moveit_task_constructor

# Check MTC version
ros2 pkg xml moveit_task_constructor_core | grep version

# Monitor task execution
ros2 topic echo /execute_task_solution

# Debug planning
ros2 topic echo /move_group/display_planned_path
```

## Configuration Files

- **Task parameters**: [config/mtc_config.yaml](config/mtc_config.yaml)
- **Launch file**: [launch/mtc_demo.launch.py](launch/mtc_demo.launch.py)
- **Demo scripts**: [scripts/](scripts/)

## Architecture Overview

```
arm_mtc (Your Task Definition)
    ↓
MoveIt Task Constructor Core
    ↓
MoveIt Move Group (Planning)
    ↓
arm_controller (Execution)
    ↓
Gazebo Simulation (or Real Hardware)
```

## Getting Help

- Full documentation: [README.md](README.md)
- MTC tutorials: https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/
- Issues: https://github.com/widemic/ldr-humanoid-arm-system/issues

## Example Task Output

When running successfully, you should see output like:

```
[INFO] [simple_move_demo]: Initializing Simple Move Task...
[INFO] [simple_move_demo]: Task pipeline configured with 5 stages
[INFO] [simple_move_demo]: Planning task...
[INFO] [simple_move_demo]: Planning succeeded! Found 1 solution(s)
[INFO] [simple_move_demo]: Executing task...
[INFO] [simple_move_demo]: Task execution succeeded!
```

Happy planning! 🤖
