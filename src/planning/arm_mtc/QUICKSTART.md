# MTC Pick-and-Place Quick Start Guide

This guide will get you up and running with the MoveIt Task Constructor pick-and-place demo in 5 minutes.

## Prerequisites

1. Workspace is built:
   ```bash
   cd ~/your_workspace
   colcon build
   source install/setup.bash
   ```

2. MTC is installed:
   ```bash
   sudo apt install ros-jazzy-moveit-task-constructor-core
   ```

## Quick Demo

### Step 1: Launch the Robot System

Open terminal 1:
```bash
# Start robot system (Gazebo + Controllers)
ros2 launch arm_system_bringup full_system.launch.py
```

Wait ~10 seconds for everything to initialize.

### Step 2: Launch MTC Demo

Open terminal 2:
```bash
# Run pick-and-place planning (includes move_group)
ros2 launch arm_mtc pick_place_demo.launch.py execute:=false
```

Wait a few seconds for move_group and the MTC node to initialize.

Open terminal 3:
```bash
# Start RViz with MoveIt config
rviz2 -d $(ros2 pkg prefix arm_moveit_config)/share/arm_moveit_config/config/moveit.rviz
```

In RViz, add **Motion Planning Tasks** display:
1. Click "Add" button
2. Select "moveit_task_constructor_msgs" → "Motion Planning Tasks"
3. Click OK

You should see:
- Console output in terminal 2: "Task planning succeeded with X solutions"
- RViz: Planned trajectory visualization
- Motion Planning Tasks panel: Stage-by-stage breakdown

### Step 4: Execute the Task (Optional)

To actually execute the planned motion, stop the current demo (Ctrl+C in terminal 2) and run:

To actually execute the planned motion:
```bash
ros2 launch arm_mtc pick_place_demo.launch.py execute:=true
```

Watch the robot perform the complete pick-and-place sequence!

## What Just Happened?

The MTC node:
1. ✅ Created a planning scene with table and target object
2. ✅ Built a task pipeline with 15+ stages
3. ✅ Planned collision-free trajectories for each stage
4. ✅ (If execute:=true) Sent trajectories to robot controllers

## Customization

### Change Object Location

Edit [config/mtc_node_params.yaml](config/mtc_node_params.yaml):

```yaml
# Object parameters
object_pose: [0.4, 0.1, 0.175, 0.0, 0.0, 0.0]  # Move object to new location
place_pose: [0.2, -0.2, 0.175, 0.0, 0.0, 0.0]   # Change place location
```

Relaunch the demo.

### Change Object Type

```yaml
object_type: "box"  # Change from "cylinder" to "box"
object_dimensions: [0.05, 0.05, 0.1]  # [x, y, z] for box
```

### Adjust Planning Behavior

```yaml
# More grasp samples (slower but more solutions)
grasp_pose_angle_delta: 0.1309  # ~7.5 degrees (smaller = more samples)

# Faster planning (less precise)
cartesian_step_size: 0.005  # Larger step size

# More solutions
max_solutions: 20
```

## Troubleshooting

### "Task planning failed"

**Common causes**:
- Object out of reach → Move `object_pose` closer
- Collision detected → Check object doesn't overlap with table
- IK solver timeout → Increase timeout in kinematics.yaml

**Debug**:
```bash
# Check robot state
ros2 topic echo /joint_states

# Check controllers
ros2 control list_controllers
```

### "No solutions found in grasp stage"

**Try**:
1. Adjust `grasp_frame_transform` in config
2. Increase `grasp_pose_max_ik_solutions`
3. Check gripper frame matches SRDF end-effector

### RViz doesn't show trajectory

**Fix**:
1. Ensure "Motion Planning Tasks" display is added
2. Check topic: `/mtc/solution` should have messages
3. Verify task is planning successfully (check console)

## Next Steps

1. **Modify the pipeline**: Edit [src/mtc_node.cpp](src/mtc_node.cpp) to add custom stages
2. **Integrate perception**: Use real-time object detection instead of static objects
3. **Add fallback strategies**: Try multiple planners (OMPL → Pilz → Cartesian)
4. **Test with real hardware**: Change `use_sim:=false` when robot is connected

## Full Documentation

See [README.md](README.md) for complete documentation including:
- Full task pipeline explanation
- All configuration parameters
- Advanced usage examples
- Troubleshooting guide

## Useful Commands

```bash
# Check MTC node is running
ros2 node list | grep mtc

# View task parameters
ros2 param list /mtc_node

# Monitor planning scene
ros2 topic echo /planning_scene

# Check joint states
ros2 topic echo /joint_states

# View controller status
ros2 control list_controllers
```

## Support

- [MTC Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [Project Documentation](../../../docs/ARCHITECTURE.md)
- [Implementation Guide](../../../mtc_implementation_guide.md)

---

**Happy Manipulating! 🦾**
