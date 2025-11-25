# arm_mtc - MoveIt Task Constructor Package

## Overview

This package provides MoveIt Task Constructor (MTC) implementations for complex manipulation tasks with the 6-DOF humanoid arm. MTC enables creation of sophisticated pick-and-place pipelines with automatic grasp pose generation, collision avoidance, and multi-stage motion planning.

## Features

- **Complete Pick and Place Pipeline**: Full implementation of cylinder grasping with 7 main stages
- **Automatic Grasp Generation**: Uses `GenerateGraspPose` to compute optimal grasps
- **Collision Handling**: Proper collision management during manipulation
- **Cartesian Planning**: Precise approach/retreat motions using Cartesian paths
- **Planning Scene Management**: Automatic addition of collision objects (cylinder + table)
- **Multiple Launch Options**: Headless or full GUI visualization

## Executables

### 1. `mtc_pick_place_cylinder` ⭐ NEW
**Complete pick and place demonstration** that grasps a cylinder and relocates it.

**Task Stages:**
1. **Current State** - Capture initial robot state
2. **Open Gripper** - Move to open position
3. **Move to Ready** - Navigate to ready pose
4. **Pick Cylinder** (Serial Container):
   - Approach object (Cartesian move downward)
   - Generate grasp pose (multiple candidates)
   - Compute IK for grasp
   - Allow hand-cylinder collision
   - Close gripper
   - Attach object
   - Lift object (Cartesian move upward)
5. **Move to Place** - Navigate to home position
6. **Place Cylinder** (Serial Container):
   - Lower object (Cartesian move downward)
   - Generate place pose
   - Compute IK for place
   - Open gripper
   - Forbid hand-cylinder collision
   - Detach object
   - Retreat (Cartesian move upward)
7. **Return Home** - Final home position

**Planning Scene:**
- **Cylinder**: 15cm height, 2.5cm radius at (0.35, 0.0, 1.075) in base_link
- **Table**: 0.8m × 0.8m × 1.0m at (0.4, 0.0, 0.5) in base_link

### 2. `mtc_node2`
Simple tutorial node that demonstrates basic MTC usage (move arm to home position).

### 3. `mtc_node`
Original complex MTC implementation (legacy).

## Launch Files

### Recommended: `mtc_pick_place_demo.launch.py` ⭐ NEW
**Primary launch method** using headless simulation with optional RViz.

```bash
# Launch with default world (headless + optional RViz)
ros2 launch arm_mtc mtc_pick_place_demo.launch.py

# Launch without RViz
ros2 launch arm_mtc mtc_pick_place_demo.launch.py use_rviz:=false

# Launch with custom world
ros2 launch arm_mtc mtc_pick_place_demo.launch.py \
  simulation_world:=/path/to/world.sdf
```

**What it launches:**
1. Full system (headless Gazebo + controllers) via `arm_system_bringup/full_system.launch.py`
2. MoveIt move_group (8s delay)
3. RViz2 with MoveIt config (10s delay, optional)
4. MTC pick and place node (12s delay)

**Timeline:**
- **0s**: Gazebo (headless) + controllers start
- **8s**: MoveIt move_group initializes
- **10s**: RViz launches (if enabled)
- **12s**: MTC demo begins execution

### Alternative: `mtc_cylinder_grasp.launch.py`
Launch using legacy full-GUI approach.

```bash
ros2 launch arm_mtc mtc_cylinder_grasp.launch.py
```

**What it launches:**
1. Complete system with Gazebo GUI + MoveIt + RViz via `moveit_gazebo.launch.py`
2. MTC pick and place node (15s delay)

**Use this if you want:**
- Full Gazebo visualization (not headless)
- Traditional launch approach
- All-in-one system startup

## Manual Launch (Step-by-Step)

For debugging or custom configurations:

```bash
# Terminal 1: Launch full system
ros2 launch arm_system_bringup full_system.launch.py

# Terminal 2: Launch MoveIt (wait 5 seconds after Terminal 1)
ros2 launch arm_moveit_config move_group.launch.py

# Terminal 3: Launch RViz (optional, wait 2 seconds after Terminal 2)
rviz2 -d $(ros2 pkg prefix arm_moveit_config)/share/arm_moveit_config/config/moveit.rviz

# Terminal 4: Run MTC demo (wait 5 seconds after Terminal 2)
ros2 run arm_mtc mtc_pick_place_cylinder
```

## Viewing Task Planning in RViz

1. In RViz, ensure these displays are enabled:
   - **MotionPlanning** - Shows robot model and planning scene
   - **Trajectory** - Visualizes planned trajectories
   - **MarkerArray** - Shows MTC stage markers

2. The MTC node publishes to `/mtc_tutorial` namespace:
   - `/mtc_tutorial/solution` - Planned solution trajectory
   - `/mtc_tutorial/task` - Task structure and stages
   - Visualization markers for each stage

3. To inspect planning:
   ```bash
   # List MTC topics
   ros2 topic list | grep mtc

   # Monitor solution publishing
   ros2 topic echo /mtc_tutorial/solution
   ```

## Configuration

### Modify Cylinder Properties

Edit [`src/mtc_pick_place_cylinder.cpp`](src/mtc_pick_place_cylinder.cpp) in `setupPlanningScene()`:

```cpp
// Change cylinder dimensions (height, radius)
cylinder.primitives[0].dimensions = { 0.15, 0.025 };

// Change cylinder position
cylinder_pose.position.x = 0.35;
cylinder_pose.position.y = 0.0;
cylinder_pose.position.z = 1.075;
```

### Modify Place Location

Edit [`src/mtc_pick_place_cylinder.cpp`](src/mtc_pick_place_cylinder.cpp) in `createTask()` under "Generate Place Pose":

```cpp
target_pose.pose.position.x = -0.1;  // Change x
target_pose.pose.position.y = 0.3;   // Change y
target_pose.pose.position.z = 1.075; // Change z
```

### Adjust Planning Parameters

```cpp
// In createTask() - Increase max solutions
if (!task_.plan(5))  // Change 5 to higher number

// In createTask() - Adjust IK solutions
wrapper->setMaxIKSolutions(8);  // Change 8 to higher/lower
wrapper->setMinSolutionDistance(1.0);  // Adjust diversity

// In createTask() - Modify approach/retreat distances
stage->setMinMaxDistance(0.1, 0.15);  // Change distances
```

## Troubleshooting

### No solutions found
- **Cause**: Target object out of reach or IK failure
- **Fix**: Adjust cylinder position closer to robot base
- **Fix**: Increase max IK solutions: `wrapper->setMaxIKSolutions(16)`

### Collision with table
- **Cause**: Approach trajectory intersects table
- **Fix**: Increase cylinder Z position
- **Fix**: Adjust table dimensions in `setupPlanningScene()`

### Execution fails after planning succeeds
- **Cause**: Controllers not properly initialized
- **Fix**: Verify controllers are active:
  ```bash
  ros2 control list_controllers
  ```
- **Fix**: Increase initialization wait time in launch file

### "Start state is out of bounds"
- **Cause**: Initial joint positions violate limits
- **Fix**: Ensure robot starts at valid pose (home or ready)
- **Fix**: Check joint limits in [`arm_moveit_config/config/joint_limits.yaml`](../arm_moveit_config/config/joint_limits.yaml)

### Gripper doesn't close properly
- **Cause**: Gripper controller timing issue
- **Fix**: This is known - uses JointInterpolationPlanner for gripper
- **Alternative**: Check hand_controller status:
  ```bash
  ros2 control list_controllers
  ```

## Architecture

### Dependencies
- `moveit_task_constructor_core` - MTC framework
- `moveit_task_constructor_msgs` - MTC message types
- `moveit_ros_planning_interface` - MoveIt planning interface
- `moveit_core` - MoveIt core libraries
- `geometry_msgs` - Geometric primitives
- `shape_msgs` - Shape definitions
- `tf2_eigen` - Eigen transformations
- `tf2_geometry_msgs` - TF2 message conversions

### Key Concepts

**Serial Containers**: Group stages that must execute sequentially (pick, place)

**Propagators**: Control information flow through pipeline:
- Forward: Current state → grasp
- Backward: Place goal → current state
- Bidirectional: Connect pick and place

**Stage Types**:
- `CurrentState` - Capture initial state
- `MoveTo` - Named pose or joint configuration
- `MoveRelative` - Relative Cartesian motion
- `GenerateGraspPose` - Compute grasp candidates
- `GeneratePlacePose` - Compute place pose
- `ComputeIK` - Inverse kinematics solver
- `ModifyPlanningScene` - Collision object manipulation

**Planners**:
- `PipelinePlanner` - OMPL-based sampling planner (arm motions)
- `CartesianPath` - Straight-line Cartesian planner (approach/retreat)
- `JointInterpolationPlanner` - Joint space interpolation (gripper)

## Development

### Building
```bash
colcon build --packages-select arm_mtc
source install/setup.bash
```

### Adding New Tasks

1. Create new C++ file in `src/`
2. Add executable in `CMakeLists.txt`:
   ```cmake
   add_executable(my_task src/my_task.cpp)
   ament_target_dependencies(my_task ...)
   install(TARGETS my_task DESTINATION lib/${PROJECT_NAME})
   ```
3. Create launch file in `launch/`
4. Build and test

### Debugging MTC Tasks

Enable verbose logging:
```bash
ros2 run arm_mtc mtc_pick_place_cylinder --ros-args --log-level debug
```

Visualize task structure:
```bash
# In another terminal after launching
ros2 topic echo /mtc_tutorial/task
```

## References

- [MoveIt Task Constructor Documentation](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [MTC GitHub Repository](https://github.com/moveit/moveit_task_constructor)
- [MTC Examples](https://github.com/moveit/moveit_task_constructor/tree/master/demo/src)

## Credits

Developed for the LDR Humanoid Arm System using ROS 2 Jazzy and MoveIt 2.

---

**Last Updated**: 2025-11-25
