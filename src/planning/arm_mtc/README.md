# arm_mtc - MoveIt Task Constructor Package

This package implements complex manipulation tasks using **MoveIt Task Constructor (MTC)** for the LDR humanoid arm system.

## Overview

MoveIt Task Constructor (MTC) provides a framework for building complex manipulation tasks by composing sequential stages. This package demonstrates a complete pick-and-place pipeline with the 6-DOF arm and 2-finger gripper.

## Features

- **Complete Pick-and-Place Pipeline**: From current state to home position
- **Intelligent Grasp Generation**: Automatic grasp pose sampling around objects
- **Collision Management**: Dynamic collision enabling/disabling for safe manipulation
- **Multiple Planning Strategies**: OMPL for global planning, Cartesian for precise motions
- **Configurable Parameters**: All tunable values in YAML config file
- **Visualization Support**: RViz integration for task inspection

## Package Structure

```
arm_mtc/
├── src/
│   └── mtc_node.cpp              # Main MTC pick-and-place implementation
├── launch/
│   └── pick_place_demo.launch.py # Launch file with MoveIt configuration
├── config/
│   └── mtc_node_params.yaml      # All task parameters
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Task Pipeline

The pick-and-place task consists of these stages:

1. **Current State** - Start from current robot configuration
2. **Open Gripper** - Open the gripper fingers
3. **Move to Pick** - Navigate arm to pick location (OMPL)
4. **Pick Object** (Serial Container):
   - Approach object (Cartesian)
   - Generate grasp poses (with IK)
   - Allow gripper-object collision
   - Close gripper
   - Allow object-surface collision
   - Attach object to gripper
   - Lift object upward (Cartesian)
   - Forbid object-surface collision
5. **Move to Place** - Navigate to place location (OMPL)
6. **Place Object** (Serial Container):
   - Lower object (Cartesian)
   - Generate place pose (with IK)
   - Open gripper
   - Forbid gripper-object collision
   - Detach object from gripper
   - Retreat from object (Cartesian)
7. **Move Home** - Return to home position (OMPL)

## Prerequisites

Before using this package, ensure you have:

1. **ROS 2 Jazzy** installed
2. **MoveIt 2** and **MoveIt Task Constructor** installed:
   ```bash
   sudo apt install ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core
   ```
3. **Workspace built** with all dependencies:
   ```bash
   cd ~/your_workspace
   colcon build
   source install/setup.bash
   ```

## Configuration

### Key Parameters (mtc_node_params.yaml)

**Robot Configuration** (must match SRDF):
- `arm_group_name`: Planning group for arm ("arm")
- `gripper_group_name`: Planning group for gripper ("hand")
- `gripper_frame`: End-effector link ("left_hand")
- `gripper_open_pose`: Named pose for open gripper ("open")
- `gripper_close_pose`: Named pose for closed gripper ("close")
- `arm_home_pose`: Named home position ("home")

**Object Configuration**:
- `object_name`: Name of target object
- `object_type`: "cylinder" or "box"
- `object_dimensions`: [height, radius] for cylinder or [x, y, z] for box
- `object_pose`: [x, y, z, roll, pitch, yaw] in world frame

**Motion Parameters**:
- `approach_object_min/max_dist`: Approach motion distance range
- `lift_object_min/max_dist`: Lift motion distance range
- `lower_object_min/max_dist`: Lower motion distance range
- `retreat_min/max_distance`: Retreat motion distance range

**Planning Parameters**:
- `execute`: Set to `true` to execute after planning
- `max_solutions`: Maximum number of solutions to find
- `grasp_pose_angle_delta`: Angular sampling for grasp generation (radians)
- `cartesian_step_size`: Step size for Cartesian planning (meters)

## Usage

### 1. Start the Robot System

First, launch the full robot system (simulation + controllers + MoveIt):

```bash
# Using GUI launcher (recommended)
ros2 run arm_gui_tools full_system_launcher.py

# Or via command line
ros2 launch arm_system_bringup full_system.launch.py
```

### 2. Launch MTC Demo

In a new terminal:

```bash
# Plan only (visualize in RViz without execution)
ros2 launch arm_mtc pick_place_demo.launch.py execute:=false

# Plan and execute
ros2 launch arm_mtc pick_place_demo.launch.py execute:=true
```

### 3. Visualize in RViz

The MTC node publishes the task solution for visualization. To view:

1. Open RViz (if not already running):
   ```bash
   rviz2 -d $(ros2 pkg prefix arm_moveit_config)/share/arm_moveit_config/config/moveit.rviz
   ```

2. Add the **Motion Planning Tasks** display:
   - Click "Add" in RViz
   - Select "Motion Planning Tasks" from the list
   - The planned trajectory will appear

3. Inspect stages:
   - Use the task panel to step through each stage
   - View solutions for each stage
   - See collision objects and trajectories

### 4. Modify Object Parameters

Edit [config/mtc_node_params.yaml](config/mtc_node_params.yaml) to change:

- Object type (cylinder/box)
- Object dimensions
- Pick/place locations
- Approach/retreat distances
- Planning parameters

Then relaunch the demo.

## Launch Arguments

- `use_sim_time` (default: true): Use simulation time
- `execute` (default: false): Execute task after planning
- `max_solutions` (default: 10): Max solutions to find

Example:
```bash
ros2 launch arm_mtc pick_place_demo.launch.py execute:=true max_solutions:=20
```

## Troubleshooting

### Task initialization fails

**Symptom**: "Task initialization failed" error

**Solutions**:
- Verify SRDF group names match parameters (`arm`, `hand`)
- Check that named poses exist in SRDF (`home`, `open`, `close`)
- Ensure end-effector `gripper_frame` matches SRDF

### Planning fails

**Symptom**: "Task planning failed" error

**Solutions**:
- Check that controllers are active: `ros2 control list_controllers`
- Verify object is reachable by arm workspace
- Increase timeout values in config file
- Try different `grasp_pose_angle_delta` (more/fewer samples)
- Check collision objects are added correctly

### No grasp solutions found

**Symptom**: Zero solutions in grasp stage

**Solutions**:
- Adjust `grasp_frame_transform` to align gripper with object
- Increase `grasp_pose_max_ik_solutions`
- Decrease `grasp_pose_min_solution_distance`
- Verify IK solver is working: check kinematics.yaml

### Execution fails

**Symptom**: Planning succeeds but execution fails

**Solutions**:
- Verify controller names match: `arm_controller`, `hand_controller`
- Check controllers accept trajectory goals
- Ensure robot is in collision-free starting pose
- Review joint limits in joint_limits.yaml

## Advanced Usage

### Modifying the Task Pipeline

To customize the task, edit [src/mtc_node.cpp](src/mtc_node.cpp):

1. Add new stages between existing ones
2. Use `Fallbacks` container for multiple planning strategies
3. Implement custom stage properties
4. Add perception integration for dynamic scenes

### Using Different Planners

The implementation uses three planners:

1. **OMPL** (`RRTConnect`): For move-to-pick, move-to-place, move-home
2. **Cartesian**: For approach, lift, lower, retreat
3. **Joint Interpolation**: For gripper open/close

To use Pilz or STOMP planners, modify the planner creation in `createTask()`.

### Integration with Perception

To use with the perception system:

1. Launch perception pipeline:
   ```bash
   ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
   ```

2. Objects detected by perception will automatically appear in planning scene

3. MTC will plan around detected obstacles

## Related Packages

- [arm_description](../../robot_description/arm_description/) - Robot URDF model
- [arm_moveit_config](../arm_moveit_config/) - MoveIt configuration
- [arm_control](../../control/arm_control/) - Simple motion control
- [arm_perception](../../perception/arm_perception/) - 3D perception

## References

- [MoveIt Task Constructor Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [MTC API Documentation](https://moveit.picknik.ai/main/api/html/namespace_moveit_1_1task__constructor.html)
- [Implementation Guide](../../../mtc_implementation_guide.md) - Complete MTC implementation reference

## License

MIT License - See repository root for details
