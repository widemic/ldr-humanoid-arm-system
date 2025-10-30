# Arm Control - Simple Motion Planner

Simple, clean interface to control the LDR arm robot in Gazebo from motion planning code.

## Quick Start

### 1. Launch Gazebo Simulation

```bash
ros2 launch arm_control sim.launch.py
```

This starts:
- Gazebo with the arm robot
- All necessary controllers

### 2. Run Example Script

In another terminal:

```bash
ros2 run arm_control example.py
```

## Usage in Your Code

```python
#!/usr/bin/env python3
import rclpy
from motion_planner import MotionPlanner

rclpy.init()
planner = MotionPlanner()

# Move to specific joint positions (5 angles in radians)
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3], duration=3.0)

# Use predefined poses
planner.home()    # [0, 0, 0, 0, 0]
planner.ready()   # [0, 1, 0, 1, 0]

# Execute a smooth trajectory with multiple waypoints
waypoints = [
    ([0.0, 0.5, 0.0, 0.5, 0.0], 2.0),   # (positions, time_from_start)
    ([0.5, 1.0, -0.5, 1.0, 0.5], 4.0),
    ([0.0, 0.0, 0.0, 0.0, 0.0], 6.0),
]
planner.move_trajectory(waypoints)

# Get current joint positions
current_pos = planner.get_position()

planner.destroy_node()
rclpy.shutdown()
```

## Joint Order

The 5 joints are (in order):
1. `base_rotation_joint` - Base rotation (roll)
2. `shoulder_pitch_joint` - Shoulder pitch
3. `elbow_pitch_joint` - Elbow pitch
4. `wrist_pitch_joint` - Wrist pitch
5. `wrist_roll_joint` - Wrist roll

## Files

- `config/controllers.yaml` - Controller configuration
- `launch/sim.launch.py` - Complete simulation launcher
- `launch/control.launch.py` - Controllers only (if Gazebo already running)
- `scripts/motion_planner.py` - Simple motion planner API
- `scripts/example.py` - Example usage

## Integration with Planning Package

To use from the planning folder:

```python
import sys
from ament_index_python.packages import get_package_share_directory
import os

# Add arm_control scripts to path
arm_control_scripts = os.path.join(
    get_package_share_directory('arm_control'),
    '../../src/control/arm_control/scripts'
)
sys.path.append(arm_control_scripts)

from motion_planner import MotionPlanner
```

Or install motion_planner as a proper Python package if needed.

## MoveIt2 Integration

You can also use MoveIt2 with this control system! See [MOVEIT_INTEGRATION.md](MOVEIT_INTEGRATION.md) for details.

**Quick start with MoveIt2:**
```bash
# Terminal 1: Start Gazebo + Controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: Start MoveIt2 with RViz
ros2 launch arm_moveit_config demo.launch.py
```

Both the simple motion planner AND MoveIt2 use the same `arm_controller`, so you can use them together or separately!

## That's It!

No complex ROS2 control knowledge needed. Just positions and trajectories.

**Choose your interface:**
- Simple motion planner for direct control
- MoveIt2 for advanced planning with collision avoidance
- Both together for maximum flexibility
