# MoveIt Task Constructor (MTC) Integration

This document describes the MoveIt Task Constructor integration for the LDR humanoid arm system.

## Overview

MoveIt Task Constructor (MTC) is a framework for defining and planning complex manipulation tasks as a sequence of reusable stages. Instead of planning single motions, MTC allows you to compose multi-step tasks like pick-and-place operations.

### Key Concepts

- **Task**: A complete manipulation sequence composed of stages
- **Stage**: A single step in the task (e.g., approach, grasp, retreat)
- **Container**: Groups stages (SerialContainer for sequential, ParallelContainer for alternatives)
- **Planner**: Solves motion for stages (Cartesian, sampling-based, interpolation)
- **Property**: Configurable parameter (group, IK frame, timeout, etc.)

## Files Created

### Configuration Files

1. **[config/mtc_capabilities.yaml](../../planning/arm_moveit_config/config/mtc_capabilities.yaml)**
   - Enables ExecuteTaskSolutionCapability in move_group
   - Defines execution and planning parameters
   - Configures Cartesian path planning settings

2. **[config/mtc_solvers.yaml](../../planning/arm_moveit_config/config/mtc_solvers.yaml)**
   - Cartesian planner configuration (step size, jump threshold)
   - Joint interpolation settings
   - OMPL and Pilz planner parameters
   - Workspace bounds

### Demo Scripts

1. **[scripts/mtc_simple_demo.py](scripts/mtc_simple_demo.py)**
   - Basic MTC demonstration
   - Task: home → approach → retreat → home
   - Good starting point for learning MTC concepts
   - No gripper required

2. **[scripts/mtc_pick_place_demo.py](scripts/mtc_pick_place_demo.py)**
   - Complete pick-and-place pipeline
   - Stages: approach → grasp → lift → move → place → retreat
   - Will be fully functional when gripper is integrated
   - Shows advanced MTC features (grasp generation, object attachment)

### Launch Files

1. **[launch/mtc_demo.launch.py](launch/mtc_demo.launch.py)**
   - Launches MTC environment with optional demo scripts
   - Supports Gazebo simulation
   - Configurable RViz visualization
   - Options: `demo:=simple|pick_place|none`

## Usage

### Quick Start

**Run the simple demo:**
```bash
# Terminal 1: Launch Gazebo + MoveIt
ros2 launch arm_control sim.launch.py

# Terminal 2: Launch MTC simple demo (wait 20s after terminal 1)
ros2 launch arm_demos mtc_demo.launch.py demo:=simple
```

### Launch Options

**All-in-one launch (Gazebo + MTC):**
```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

**MTC environment only (for custom scripts):**
```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=none
```

**Run demo scripts directly:**
```bash
# After launching Gazebo and MoveIt separately
ros2 run arm_demos mtc_simple_demo.py
ros2 run arm_demos mtc_pick_place_demo.py
```

### Launch Arguments

- `demo` - Which demo to run: `simple`, `pick_place`, or `none` (default: `simple`)
- `use_sim` - Use Gazebo simulation: `true`/`false` (default: `false`)
- `use_rviz` - Launch RViz: `true`/`false` (default: `true`)
- `log_level` - Logging level: `DEBUG`, `INFO`, `WARN`, `ERROR` (default: `INFO`)

## MTC Stage Types

### Core Stages

- **CurrentState** - Captures current robot state as starting point
- **FixedState** - Uses a predefined robot state
- **ComputeIK** - Computes inverse kinematics for a target pose

### Motion Stages

- **MoveTo** - Move to named pose or joint configuration
- **MoveRelative** - Move relative to current pose (Cartesian or joint)
- **Connect** - Connect two states with motion planning

### Planning Scene Modification

- **ModifyPlanningScene** - Modify collision environment
  - Attach/detach objects
  - Allow/forbid collisions
  - Add/remove collision objects

### Grasp/Place Stages

- **GenerateGraspPose** - Generate candidate grasp poses
- **GeneratePlacePose** - Generate candidate place poses
- **SimpleGrasp** - Simple parallel-jaw grasp generator

### Containers

- **SerialContainer** - Execute stages sequentially
- **Alternatives** (ParallelContainer) - Try multiple approaches
- **Fallbacks** - Fallback to alternative if primary fails
- **Merger** - Merge multiple solution paths

## Creating Custom Tasks

### Basic Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Vector3Stamped

class MyTask(Node):
    def __init__(self):
        super().__init__('my_task_node')

        # Create task
        self.task = core.Task("my_task")
        self.task.loadRobotModel(self.get_logger())

        # Set properties
        self.task.setProperty("group", "arm")
        self.task.setProperty("ik_frame", "wrist_roll_link")

        # Create planners
        self.cartesian = core.CartesianPath()
        self.cartesian.setMaxVelocityScalingFactor(0.5)
        self.cartesian.setStepSize(0.01)

        self.sampling = core.PipelinePlanner()

        # Build task
        self.build_task()

    def build_task(self):
        # Start from current state
        current = stages.CurrentState("current")
        self.task.add(current)

        # Add your stages here
        # ...

    def plan(self):
        self.get_logger().info("Planning task...")
        self.task.plan()

        if self.task.numSolutions() > 0:
            self.get_logger().info(f"Found {self.task.numSolutions()} solutions")
            return True
        else:
            self.get_logger().error("No solutions found")
            return False

    def execute(self):
        if self.task.numSolutions() > 0:
            self.get_logger().info("Executing best solution...")
            self.task.execute(self.task.solutions()[0])
            return True
        return False

def main():
    rclpy.init()
    task = MyTask()

    if task.plan():
        response = input("Execute? (y/n): ")
        if response.lower() == 'y':
            task.execute()

    task.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Stage Configuration Tips

1. **Set group for each stage:**
   ```python
   stage.setGroup("arm")
   ```

2. **Configure IK frame:**
   ```python
   stage.setIKFrame("wrist_roll_link")
   ```

3. **Set timeout:**
   ```python
   stage.setTimeout(5.0)  # seconds
   ```

4. **Configure inheritance:**
   ```python
   stage.properties().configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
   ```

5. **Set velocity/acceleration scaling:**
   ```python
   cartesian.setMaxVelocityScalingFactor(0.5)
   cartesian.setMaxAccelerationScalingFactor(0.5)
   ```

## Common Patterns

### Move to Named Pose

```python
move_home = stages.MoveTo("move home", sampling_planner)
move_home.setGroup("arm")
move_home.setGoal("home")  # From SRDF
task.add(move_home)
```

### Move Relative (Cartesian)

```python
approach = stages.MoveRelative("approach", cartesian_planner)
approach.setGroup("arm")
approach.setMinMaxDistance(0.1, 0.2)

direction = Vector3Stamped()
direction.header.frame_id = "wrist_roll_link"
direction.vector.x = 1.0  # Forward
approach.setDirection(direction)

task.add(approach)
```

### Connect Two States

```python
connect = stages.Connect(
    "connect",
    [("arm", sampling_planner)]
)
connect.properties().configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
task.add(connect)
```

### Modify Planning Scene

```python
# Attach object
attach = stages.ModifyPlanningScene("attach object")
attach.attachObject("object_name", "wrist_roll_link")
task.add(attach)

# Allow collision
allow = stages.ModifyPlanningScene("allow collision")
allow.allowCollisions("object_name", ["wrist_roll_link"], True)
task.add(allow)
```

## Debugging

### Print Task Structure

```python
self.get_logger().info(self.task.toString())
```

### Check Solutions

```python
if self.task.numSolutions() > 0:
    best = self.task.solutions()[0]
    print(f"Best solution cost: {best.cost()}")
```

### Enable Debug Logging

```bash
ros2 launch arm_demos mtc_demo.launch.py log_level:=DEBUG
```

### Visualize in RViz

1. Add "Motion Planning Tasks" panel in RViz
2. Select your task from the dropdown
3. Visualize individual stages
4. Inspect solution trajectories

## Dependencies

The following ROS 2 packages are required:

- `moveit_task_constructor_core` - Core MTC library
- `moveit_task_constructor_msgs` - MTC message definitions
- `moveit_task_constructor_capabilities` - move_group capabilities
- `moveit_task_constructor_visualization` - RViz plugins

All dependencies are declared in [package.xml](../../../planning/arm_moveit_config/package.xml) and [arm_demos/package.xml](package.xml).

## Installation

```bash
# Install MTC packages (if not already installed)
sudo apt install ros-jazzy-moveit-task-constructor-*

# Build the workspace
cd ~/your_workspace
colcon build --packages-select arm_moveit_config arm_demos
source install/setup.bash
```

## Troubleshooting

### "No solutions found"

1. Check joint limits in `joint_limits.yaml`
2. Increase planning time: `sampling_planner.setTimeout(10.0)`
3. Reduce velocity scaling for tighter tolerances
4. Check for collision issues in planning scene
5. Verify IK frame is correct

### "Task execution failed"

1. Ensure controllers are active: `ros2 control list_controllers`
2. Check that move_group has ExecuteTaskSolutionCapability loaded
3. Verify trajectory execution parameters in `mtc_capabilities.yaml`
4. Check for controller timeout issues

### "Stage XYZ failed"

1. Use `task.toString()` to inspect task structure
2. Check stage-specific properties (group, IK frame, timeout)
3. Verify planner configuration in `mtc_solvers.yaml`
4. Enable debug logging to see detailed errors

## Future Work

- [ ] Integration with gripper/hand for complete pick-and-place
- [ ] Object detection integration for autonomous grasping
- [ ] Grasp database for common objects
- [ ] Multi-arm coordination with MTC
- [ ] Task learning and optimization

## References

- [MTC Documentation](https://ros-planning.github.io/moveit_task_constructor/)
- [MTC Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html)
- [MTC GitHub](https://github.com/moveit/moveit_task_constructor)
- [MoveIt 2 Documentation](https://moveit.ros.org/)

## License

MIT License - See main repository LICENSE file
