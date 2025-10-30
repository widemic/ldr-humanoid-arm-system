# MoveIt2 Integration Guide

Yes! You can use the MoveIt2 demo launch from `arm_moveit_config` with the refactored control system.

## How It Works

The MoveIt2 configuration already points to the same `arm_controller` that we use:

```yaml
# From arm_moveit_config/config/moveit_controllers.yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller  # ← Same controller!

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
```

This means MoveIt2 will use the same controller interface as our simple motion planner.

## Usage Options

### Option 1: MoveIt2 Demo (Planning + Visualization)

Launch Gazebo simulation + MoveIt2 with RViz:

```bash
# Terminal 1: Start Gazebo simulation with controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: Start MoveIt2 demo (planning + RViz)
ros2 launch arm_moveit_config demo.launch.py
```

**What you get:**
- Gazebo simulation (physics)
- MoveIt2 move_group node (motion planning)
- RViz with interactive markers (drag & plan)
- Full planning capabilities (collision avoidance, etc.)

**Use RViz to:**
- Drag the interactive marker to set goal pose
- Click "Plan" to generate trajectory
- Click "Execute" to send to Gazebo
- Use predefined poses from planning groups

### Option 2: MoveIt2 + Simple Motion Planner

Use both MoveIt2 for planning AND the simple motion planner programmatically:

```bash
# Terminal 1: Gazebo + Controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: MoveIt2 (optional - for visualization)
ros2 launch arm_moveit_config demo.launch.py

# Terminal 3: Your planning code
ros2 run arm_control example.py
# OR your custom planning node
```

**What you get:**
- MoveIt2 for advanced planning (obstacle avoidance, IK, etc.)
- Simple motion planner for direct control
- Both use the same `arm_controller` underneath
- No conflicts - they share the same action interface

### Option 3: Simple Motion Planner Only

Just use the simple API without MoveIt2:

```bash
# Terminal 1: Gazebo + Controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: Your code
ros2 run arm_control example.py
```

**What you get:**
- Lightweight setup
- Direct trajectory control
- No collision checking
- Fast and simple

## Programming with MoveIt2

### Using MoveIt2 Python API

```python
#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

rclpy.init()

# Initialize MoveIt
moveit = MoveItPy(node_name="moveit_demo")
arm = moveit.get_planning_component("arm")

# Set goal from predefined pose
arm.set_goal(pose_name="home")

# Plan
plan_result = arm.plan()

# Execute (sends to arm_controller in Gazebo)
if plan_result:
    arm.execute()
```

### Combining MoveIt2 + Simple Motion Planner

```python
#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
import sys
sys.path.append('/path/to/arm_control/scripts')
from motion_planner import MotionPlanner

rclpy.init()

# Option A: Use MoveIt for complex planning
moveit = MoveItPy(node_name="hybrid_planner")
arm_planning = moveit.get_planning_component("arm")

# Plan with collision avoidance
arm_planning.set_goal(pose_name="ready")
plan = arm_planning.plan()
if plan:
    arm_planning.execute()

# Option B: Use simple planner for direct moves
planner = MotionPlanner()
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3])
```

## Architecture: How They Work Together

```
┌─────────────────────────────────┐
│  MoveIt2 (move_group)           │
│  - Motion planning              │
│  - Collision detection          │
│  - IK solving                   │
└──────────┬──────────────────────┘
           │
           │ FollowJointTrajectory action
           │
           ▼
┌─────────────────────────────────┐ ◄──── Simple Motion Planner
│  arm_controller                 │       (direct trajectories)
│  (JointTrajectoryController)    │
└──────────┬──────────────────────┘
           │
           │ Position commands
           ▼
┌─────────────────────────────────┐
│  Gazebo Simulation              │
│  (gz_ros2_control)              │
└─────────────────────────────────┘
```

**Key Point:** Both MoveIt2 and the simple motion planner send trajectories to the same `arm_controller`. They can coexist!

## When to Use What?

### Use MoveIt2 Demo When:
- ✅ You need motion planning with obstacle avoidance
- ✅ You want interactive visualization in RViz
- ✅ You need inverse kinematics (Cartesian planning)
- ✅ You're developing/testing complex behaviors
- ✅ You want to use predefined planning scenes

### Use Simple Motion Planner When:
- ✅ You have pre-computed trajectories
- ✅ You want direct, fast control
- ✅ You don't need collision checking
- ✅ You're doing low-level control experiments
- ✅ You want minimal dependencies

### Use Both When:
- ✅ Develop with MoveIt2 RViz visualization
- ✅ Test/validate with simple motion planner
- ✅ Plan complex motions with MoveIt2
- ✅ Execute simple sequences with motion planner

## Quick Start Examples

### Example 1: MoveIt2 Demo in RViz

```bash
# Terminal 1
ros2 launch arm_control sim.launch.py

# Terminal 2
ros2 launch arm_moveit_config demo.launch.py
```

Then in RViz:
1. Enable "MotionPlanning" display
2. In Planning tab → drag interactive marker
3. Click "Plan & Execute"
4. Watch robot move in Gazebo!

### Example 2: MoveIt2 Python Script

```bash
# Terminal 1
ros2 launch arm_control sim.launch.py

# Terminal 2
ros2 launch arm_moveit_config move_group.launch.py

# Terminal 3
python3 your_moveit_script.py
```

### Example 3: Simple Motion Planner

```bash
# Terminal 1
ros2 launch arm_control sim.launch.py

# Terminal 2
ros2 run arm_control example.py
```

## Troubleshooting

### "Controller not available"
Make sure `arm_control sim.launch.py` is running first. Wait ~3 seconds for controllers to spawn.

### "Planning scene not updating"
If using MoveIt2, ensure robot_state_publisher is running (it's included in demo.launch.py).

### Both systems sending commands?
They share the action interface - only one should send goals at a time. The last goal sent wins.

## Configuration Files

The integration works because these files are aligned:

**Control side:**
- `arm_control/config/controllers.yaml` - Defines arm_controller

**MoveIt2 side:**
- `arm_moveit_config/config/moveit_controllers.yaml` - Points to arm_controller
- `arm_moveit_config/config/ros2_controllers.yaml` - Same controller definitions

They all reference the same controller: `/arm_controller/follow_joint_trajectory`

## Summary

✅ **YES** - You can use the MoveIt2 demo launch!

✅ **YES** - It works with the refactored control system!

✅ **YES** - You can use both MoveIt2 AND the simple motion planner!

The refactored control system is fully compatible with MoveIt2. Use whichever tool fits your needs, or use both together!
