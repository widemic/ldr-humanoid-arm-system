# Quick Start Guide

## Three Ways to Control the Arm

### 1️⃣ Simple Motion Planner (Easiest)

**When:** You want direct, programmatic control

```bash
# Terminal 1: Start simulation
ros2 launch arm_control sim.launch.py

# Terminal 2: Run your code
ros2 run arm_control example.py
```

**Code:**
```python
from motion_planner import MotionPlanner

planner = MotionPlanner()
planner.move_to([0, 1, 0, 1, 0])
planner.home()
```

---

### 2️⃣ MoveIt2 Demo (Interactive)

**When:** You want interactive planning with RViz visualization

```bash
# Terminal 1: Start simulation
ros2 launch arm_control sim.launch.py

# Terminal 2: Start MoveIt2
ros2 launch arm_moveit_config demo.launch.py
```

**Then:** Use RViz interactive markers to drag and plan motions!

---

### 3️⃣ Both Together (Best of Both Worlds)

**When:** You want planning + direct control

```bash
# Terminal 1: Simulation
ros2 launch arm_control sim.launch.py

# Terminal 2: MoveIt2 for visualization
ros2 launch arm_moveit_config demo.launch.py

# Terminal 3: Your planning code
python3 my_planner.py  # Uses MotionPlanner or MoveIt2 API
```

---

## File Reference

| What | Where |
|------|-------|
| 📘 Simple API guide | [README.md](README.md) |
| 🔧 MoveIt2 integration | [MOVEIT_INTEGRATION.md](MOVEIT_INTEGRATION.md) |
| 📝 Refactor details | [REFACTOR_SUMMARY.md](REFACTOR_SUMMARY.md) |
| 💻 Example code | [scripts/example.py](scripts/example.py) |
| 🧪 Quick test | [scripts/test_simple.py](scripts/test_simple.py) |
| 🎮 Motion planner API | [scripts/motion_planner.py](scripts/motion_planner.py) |

---

## One-Liner Tests

```bash
# Test 1: Launch simulation
ros2 launch arm_control sim.launch.py

# Test 2: Run example (in new terminal)
ros2 run arm_control example.py

# Test 3: Quick verification
ros2 run arm_control test_simple.py
```

---

## Common Commands

```bash
# List active controllers
ros2 control list_controllers

# Check joint states
ros2 topic echo /joint_states

# Check controller state
ros2 topic echo /arm_controller/state

# Send manual trajectory
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [...], points: [...]}}"
```

---

## Architecture at a Glance

```
Your Code (Planning)
       ↓
┌──────────────┐     ┌──────────────┐
│ Simple API   │ OR  │   MoveIt2    │
│ MotionPlanner│     │  move_group  │
└──────┬───────┘     └──────┬───────┘
       └──────────┬──────────┘
                  ↓
          arm_controller
         (JointTrajectory)
                  ↓
            Gazebo Sim
```

Both paths work! Choose based on your needs.

---

## Next Steps

1. ✅ Read [README.md](README.md) for simple API details
2. ✅ Check [MOVEIT_INTEGRATION.md](MOVEIT_INTEGRATION.md) for MoveIt2 usage
3. ✅ Study [scripts/example.py](scripts/example.py) for code examples
4. ✅ Build your own planner!
