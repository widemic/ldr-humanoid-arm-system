# MTC Quick Start Guide

**MoveIt Task Constructor (MTC) Integration for LDR Humanoid Arm**

## TL;DR - Get Started in 3 Steps

### 1. Build & Check

```bash
cd /path/to/ldr-humanoid-arm-system
colcon build --packages-select arm_moveit_config arm_demos
source install/setup.bash
ros2 run arm_demos check_mtc.py
```

### 2. Install MTC (if needed)

```bash
sudo apt install ros-jazzy-moveit-task-constructor-*
source /opt/ros/jazzy/setup.bash
```

### 3. Run Demo

```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

---

## What is MTC?

MoveIt Task Constructor lets you build complex manipulation tasks from simple stages:

```python
# Instead of planning one motion at a time...
moveit.plan_to_pose(target1)
moveit.execute()
moveit.plan_to_pose(target2)
moveit.execute()

# ...compose a complete task:
task = Task()
task.add(MoveTo("home"))
task.add(MoveRelative("approach", direction=forward))
task.add(MoveRelative("retreat", direction=backward))
task.plan()  # Plans entire sequence
task.execute()  # Executes all steps
```

## Files Created

| File | Purpose |
|------|---------|
| `check_mtc.py` | Check if MTC is installed |
| `mtc_simple_demo.py` | Basic approach-retreat demo |
| `mtc_pick_place_demo.py` | Advanced pick-and-place (needs gripper) |
| `mtc_demo.launch.py` | Launch file for MTC demos |
| `mtc_capabilities.yaml` | MTC configuration for move_group |
| `mtc_solvers.yaml` | Stage planner configurations |

## Common Commands

```bash
# Check MTC installation
ros2 run arm_demos check_mtc.py

# Run simple demo (with Gazebo)
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true

# Run without Gazebo (requires real hardware or separate sim)
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=false

# Launch MTC environment only (no demo script)
ros2 launch arm_demos mtc_demo.launch.py demo:=none

# Run demo script directly (after launching MoveIt separately)
ros2 run arm_demos mtc_simple_demo.py
```

## Troubleshooting

**"MTC not installed" warning?**
```bash
sudo apt install ros-jazzy-moveit-task-constructor-*
source /opt/ros/jazzy/setup.bash
```

**Demo script crashes?**
```bash
# Make sure move_group is running
ros2 node list | grep move_group

# Check controllers are active
ros2 control list_controllers

# Verify robot model loaded
ros2 topic echo /joint_states --once
```

**"No solutions found"?**
- Increase planning timeout in script
- Check joint limits in `config/joint_limits.yaml`
- Verify IK frame is correct: `wrist_roll_link`

## Learn More

- **Full Guide:** [MTC_INTEGRATION.md](MTC_INTEGRATION.md)
- **Detailed Docs:** [src/applications/arm_demos/README_MTC.md](src/applications/arm_demos/README_MTC.md)
- **System Docs:** [CLAUDE.md](CLAUDE.md) (see MTC section)
- **Official MTC Docs:** https://ros-planning.github.io/moveit_task_constructor/

## Demo Examples

### Simple Approach-Retreat

```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

**What it does:**
1. Moves arm to home position
2. Approaches forward 10-20cm (Cartesian)
3. Retreats backward 10-20cm (Cartesian)
4. Returns to home position

### Pick-and-Place (Future)

```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=pick_place use_sim:=true
```

**Status:** Requires gripper integration (coming soon)

## Architecture

```
User Script
    ↓
MTC Task (stages)
    ↓
Planners (Cartesian/OMPL/Interpolation)
    ↓
MoveIt move_group
    ↓
arm_controller
    ↓
Gazebo/Hardware
```

## Next Steps

1. ✅ Run `check_mtc.py` to verify installation
2. ✅ Test simple demo with Gazebo
3. ✅ Read [README_MTC.md](src/applications/arm_demos/README_MTC.md) for patterns
4. ✅ Create your own custom tasks
5. ⏳ Wait for gripper integration for pick-and-place

---

**Questions?** See [MTC_INTEGRATION.md](MTC_INTEGRATION.md) for detailed documentation.
