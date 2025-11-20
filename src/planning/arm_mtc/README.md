# arm_mtc - MoveIt Task Constructor Package

MoveIt Task Constructor (MTC) package for complex task planning with the 5-DOF humanoid arm. MTC provides a flexible framework for composing complex manipulation tasks from reusable building blocks (stages).

## Overview

MTC is designed for:
- **Complex manipulation tasks** (pick and place, assembly, etc.)
- **Task-level planning** beyond simple motion planning
- **Reusable task definitions** with modular stages
- **Multiple solution generation** for robust execution

## Features

- Pick and place task demonstrations
- Modular stage-based task composition
- Integration with arm MoveIt configuration
- Configurable approach/retreat behaviors
- Support for custom task pipelines

## Prerequisites

### Required Dependencies
```bash
sudo apt install ros-jazzy-moveit-task-constructor-core \
                 ros-jazzy-moveit-task-constructor-msgs \
                 ros-jazzy-moveit-task-constructor-visualization
```

### System Requirements
- ROS 2 Jazzy
- MoveIt2
- Gazebo Harmonic (for simulation)
- arm_moveit_config package

## Building

```bash
# From repository root
colcon build --packages-select arm_mtc
source install/setup.bash
```

## Usage

### Running the Pick and Place Demo

**Step 1: Launch Gazebo simulation**
```bash
# Terminal 1
ros2 launch arm_control sim.launch.py
```

**Step 2: Launch MoveIt (wait ~20 seconds after Step 1)**
```bash
# Terminal 2
ros2 launch arm_moveit_config demo.launch.py
```

**Step 3: Launch MTC demo (wait until MoveIt is ready)**
```bash
# Terminal 3
ros2 launch arm_mtc mtc_demo.launch.py
```

### Visualizing Tasks in RViz

The MTC visualization plugin can be added to RViz to inspect task solutions:
1. In RViz, add "Motion Planning Tasks" panel
2. Select the task from the dropdown
3. Browse through computed solutions
4. Inspect individual stages

## Package Structure

```
arm_mtc/
├── config/
│   └── mtc_config.yaml          # Task configuration parameters
├── launch/
│   └── mtc_demo.launch.py       # Demo launch file
├── scripts/
│   └── pick_place_demo.py       # Pick and place demonstration
├── arm_mtc/
│   └── __init__.py              # Python package init
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Configuration

Edit [config/mtc_config.yaml](config/mtc_config.yaml) to customize:
- Planning parameters (timeouts, planners)
- Workspace bounds
- Approach/retreat distances and directions
- Object dimensions and locations
- Frame IDs

## Creating Custom Tasks

### Basic Task Structure

```python
from moveit.task_constructor import core, stages

# Create task
task = core.Task()
task.name = "my_custom_task"

# Add stages
current = stages.CurrentState("current state")
task.add(current)

move = stages.MoveTo("move to pose", "arm")
move.setGoal("home")
task.add(move)

# Plan and execute
task.plan()
task.execute()
```

### Common Stage Types

- **CurrentState**: Start from current robot state
- **MoveTo**: Move to named pose or joint state
- **MoveRelative**: Move relative to current pose
- **Connect**: Connect two states with motion plan
- **GeneratePose**: Generate grasp/place poses
- **ModifyPlanningScene**: Add/remove collision objects

## Integration with Arm System

This package integrates with:
- **arm_description**: Robot URDF model
- **arm_moveit_config**: Planning group "arm", kinematics, SRDF
- **arm_control**: Controller interface for execution
- **arm_gazebo**: Simulation environment

## Task Stages Explained

The pick and place demo includes these stages:

1. **Current State** - Start from current configuration
2. **Move to Pick** - Navigate to pre-grasp location
3. **Approach** - Move end-effector toward object
4. **Grasp** - Close gripper (placeholder for future)
5. **Attach** - Attach object to end-effector
6. **Lift** - Lift object vertically
7. **Move to Place** - Transport object to place location
8. **Lower** - Lower object to surface
9. **Detach** - Detach object from end-effector
10. **Release** - Open gripper
11. **Retreat** - Move away from object
12. **Return Home** - Return to home position

## Troubleshooting

**Task planning fails:**
- Ensure MoveIt move_group is running
- Check that "arm" planning group exists in SRDF
- Verify joint limits in configuration
- Increase planning timeout in config

**No solutions found:**
- Check workspace bounds in config
- Verify start and goal poses are reachable
- Review approach/retreat distances
- Check for collision constraints

**Execution fails:**
- Verify controllers are active: `ros2 control list_controllers`
- Check controller configuration matches MoveIt
- Ensure simulation time is synchronized

## Examples

### Simple Movement Task

```python
#!/usr/bin/env python3
import rclpy
from moveit.task_constructor import core, stages

rclpy.init()

task = core.Task()
task.name = "simple_move"

# Start from current state
task.add(stages.CurrentState("current"))

# Move to home position
move = stages.MoveTo("go home", "arm")
move.setGoal("home")
task.add(move)

# Plan and execute
if task.plan():
    task.execute()

rclpy.shutdown()
```

## Future Enhancements

- Gripper integration for actual grasping
- Multiple grasp pose generation
- Collision object management
- Custom constraint definitions
- Task serialization/deserialization
- Multi-arm coordination tasks

## References

- [MoveIt Task Constructor Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [MTC GitHub Repository](https://github.com/moveit/moveit_task_constructor)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

## License

MIT License

## Maintainer

andrei-dragomir <widemic@gmail.com>
