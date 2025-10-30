# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS 2 Jazzy** robotics control system for a **5-DOF humanoid arm**. The system provides both simple direct control and advanced MoveIt2 motion planning capabilities, with full Gazebo Harmonic simulation support.

**Tech Stack:**
- ROS 2 Jazzy on Ubuntu 24.04 (Noble)
- Gazebo Harmonic for physics simulation
- MoveIt2 for motion planning
- ros2_control framework for controller management
- Python 3 + C++ with ament_cmake build system

**Repository:** https://github.com/widemic/ldr-humanoid-arm-system

## Robot Specifications

**5 Actuated Joints:**
1. `base_rotation_joint` - Base rotation (-3.14 to 3.14 rad, 120 Nm, 3.0 rad/s)
2. `shoulder_pitch_joint` - Shoulder pitch (-0.55 to 3.1 rad, 120 Nm, 3.0 rad/s)
3. `elbow_pitch_joint` - Elbow pitch (-3.14 to 3.14 rad, 60 Nm, 1.6 rad/s)
4. `wrist_pitch_joint` - Wrist pitch (-0.31 to 2.8 rad, 60 Nm, 1.6 rad/s)
5. `wrist_roll_joint` - Wrist roll (-3.14 to 3.14 rad, 17 Nm, 0.8 rad/s)

## Build System

**Building the project:**
```bash
# From repository root
colcon build
source install/setup.bash
```

**Incremental builds:**
```bash
# Build specific package
colcon build --packages-select arm_control

# Build with verbose output
colcon build --event-handlers console_direct+
```

**Clean build:**
```bash
rm -rf build/ install/ log/
colcon build
```

## Running the System

### Modular Launch (Recommended - Separate Terminals)

This approach launches components separately for better debugging and modularity:

```bash
# Terminal 1: Launch Gazebo simulation + controllers
ros2 launch arm_control sim.launch.py

# Terminal 2: Launch MoveIt planning + RViz (wait ~20s after terminal 1)
ros2 launch arm_moveit_config demo.launch.py
```

**What each launch does:**
- **sim.launch.py** - Gazebo world + robot + joint_state_broadcaster + arm_controller
- **demo.launch.py** - robot_state_publisher + MoveIt move_group + RViz

### Individual Component Launch

For maximum control, launch each component separately:

```bash
# Terminal 1: Gazebo world only
ros2 launch arm_gazebo arm_world.launch.py

# Terminal 2: Spawn robot + controllers (wait 5s)
ros2 launch arm_gazebo spawn_arm.launch.py

# Terminal 3: MoveIt planning without RViz (wait 15s)
ros2 launch arm_moveit_config demo.launch.py use_rviz:=false

# Terminal 4: RViz separately if needed
ros2 launch arm_moveit_config demo.launch.py
```

### All-in-One Launch (Alternative)

If you prefer everything in one command:

```bash
# Complete system: Gazebo + MoveIt + RViz
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Without RViz (headless)
ros2 launch arm_system_bringup moveit_gazebo.launch.py use_rviz:=false
```

**Initialization sequence (all-in-one):**
- t=0s: Gazebo world + robot spawn + robot_state_publisher
- t=3s: joint_state_broadcaster spawned
- t=4s: arm_controller spawned
- t=6s: move_group planning server started
- t=8s: RViz launched
- System ready in ~10 seconds

## Testing

**Quick verification tests:**
```bash
# Run example motion sequence
ros2 run arm_control example.py

# Simple test
ros2 run arm_control test_simple.py
```

**Check system status:**
```bash
# List active controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check controller manager status
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## Package Architecture

### Active Packages (Functional)

**[arm_description](src/robot_description/arm_description/)** - Robot URDF/Xacro model
- Main: `urdf/arm.urdf.xacro` (switches between sim/real via `use_sim` argument)
- Links: `urdf/links/arm_links.xacro` (14 links with inertia)
- Joints: `urdf/joints/arm_joints.xacro` (5 revolute + 8 fixed)
- Control: `urdf/macros/ros2_control.xacro` (ros2_control interface)
- Meshes: 14 STL files each for visual and collision

**[arm_control](src/control/arm_control/)** - Simple control interface
- Simple motion planner API: `scripts/motion_planner.py`
- Controllers config: `config/controllers.yaml`
- Launch: `launch/sim.launch.py` (full simulation), `launch/control.launch.py` (controllers only)

**[arm_moveit_config](src/planning/arm_moveit_config/)** - MoveIt2 planning
- SRDF: `config/arm_description.srdf` (planning group "arm", predefined poses)
- Kinematics: `config/kinematics.yaml` (KDL solver, 50ms timeout)
- Launch: `launch/demo.launch.py` (MoveIt + RViz), `launch/move_group.launch.py` (planning only)

**[arm_gazebo](src/simulation/arm_gazebo/)** - Gazebo Harmonic simulation
- World: `launch/arm_world.launch.py` + `worlds/lab.sdf`
- Spawn: `launch/spawn_arm.launch.py` (robot + controllers)
- Config: `config/controllers.yaml` (Gazebo-specific controller settings)

**[arm_system_bringup](src/bringup/arm_system_bringup/)** - System integration
- Complete demo: `launch/moveit_gazebo.launch.py` (Gazebo + MoveIt + RViz)

### Placeholder Packages (Not Yet Implemented)
- Hardware interfaces (`arm_hardware`)
- Gripper/hand packages (`gripper_*`, `hand_*`)
- Dual-arm configurations (`dual_arm_*`)
- GUI/diagnostic tools

## Control Architecture

### Two Control Paths

**Path 1: Simple Motion Planner (arm_control)**
```python
from motion_planner import MotionPlanner

planner = MotionPlanner()
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3])  # 5 joint positions
planner.home()  # Move to home position
```
- Direct trajectory control, no MoveIt overhead
- Fast and lightweight
- Good for known trajectories

**Path 2: MoveIt2 Planning (arm_moveit_config)**
```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="planner")
arm = moveit.get_planning_component("arm")
arm.set_goal(pose_name="home")
plan = arm.plan()
if plan:
    arm.execute()
```
- Collision avoidance and inverse kinematics
- Cartesian planning
- Interactive RViz visualization
- Predefined poses (home, etc.)

**Both paths share the same controller interface:** `arm_controller/follow_joint_trajectory` action

### Controller Stack

```
User Code → arm_controller (JointTrajectoryController)
         → GazeboSimROS2ControlPlugin
         → Gazebo Physics Engine
         → joint_state_broadcaster (publishes /joint_states)
```

**Controllers defined in config/controllers.yaml:**
- `joint_state_broadcaster` - Publishes joint states at 50 Hz
- `arm_controller` - JointTrajectoryController with position/velocity interfaces

## Important Configuration Files

**Controller configuration:**
- [src/control/arm_control/config/controllers.yaml](src/control/arm_control/config/controllers.yaml)
- [src/simulation/arm_gazebo/config/controllers.yaml](src/simulation/arm_gazebo/config/controllers.yaml) (same)

**MoveIt planning:**
- [src/planning/arm_moveit_config/config/kinematics.yaml](src/planning/arm_moveit_config/config/kinematics.yaml) - IK solver (KDL, 50ms timeout)
- [src/planning/arm_moveit_config/config/joint_limits.yaml](src/planning/arm_moveit_config/config/joint_limits.yaml) - Planning limits
- [src/planning/arm_moveit_config/config/arm_description.srdf](src/planning/arm_moveit_config/config/arm_description.srdf) - Planning group, poses, collision pairs

**Robot model:**
- [src/robot_description/arm_description/urdf/arm.urdf.xacro](src/robot_description/arm_description/urdf/arm.urdf.xacro) - Main entry (use_sim:=true/false)
- [src/robot_description/arm_description/urdf/macros/ros2_control.xacro](src/robot_description/arm_description/urdf/macros/ros2_control.xacro) - Control interface

## Development Guidelines

### Working with URDF/Xacro
- The system supports two modes via `use_sim` argument:
  - `use_sim:=true` → Loads [arm_gazebo.xacro](src/robot_description/arm_description/urdf/arm_gazebo.xacro) (GazeboSimROS2ControlPlugin)
  - `use_sim:=false` → Loads [arm_real.xacro](src/robot_description/arm_description/urdf/arm_real.xacro) (for future hardware)
- Always test URDF changes in both Gazebo and RViz
- Run `check_urdf` on generated URDF files to validate

### Working with Controllers
- Controllers need delays for proper initialization (see launch files)
- Controller manager needs ~5s to initialize in Gazebo
- Always spawn `joint_state_broadcaster` before `arm_controller`
- Use `ros2 control list_controllers` to verify controller state

### Working with MoveIt
- Planning group "arm" includes all 5 joints
- KDL kinematics solver is default (TRAC-IK available as alternative)
- SRDF defines ~40+ disabled collision pairs for performance
- Test planning with RViz interactive markers before coding

### Working with Launch Files
- Use TimerAction for sequential initialization (see [moveit_gazebo.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo.launch.py))
- Always set `use_sim_time: True` for Gazebo nodes
- Gazebo Harmonic uses `gz_sim` (not `gazebo` legacy)
- Clock bridge synchronizes ROS time with Gazebo time

## Key Architectural Decisions

1. **Modular package structure** - Control, planning, simulation, and description are separate for flexibility
2. **Two control paths** - Simple API for basic control, MoveIt for complex planning
3. **Shared controller interface** - Both paths use same `arm_controller` for consistency
4. **Simulation-first design** - Use `use_sim` flag to switch between simulation and real hardware
5. **Timer-based initialization** - Ensures proper startup sequence in complex launch files

## Common Issues and Solutions

**Controllers not spawning:**
- Check that Gazebo has fully initialized (wait 5+ seconds)
- Verify `ros2 control list_controllers` shows controller_manager
- Ensure ros2_control plugin is loaded in URDF (check Gazebo logs)

**MoveIt planning fails:**
- Verify controllers are active: `ros2 control list_controllers`
- Check joint limits in [joint_limits.yaml](src/planning/arm_moveit_config/config/joint_limits.yaml)
- Increase IK solver timeout in [kinematics.yaml](src/planning/arm_moveit_config/config/kinematics.yaml)

**Gazebo simulation not starting:**
- Ensure Gazebo Harmonic is installed (not legacy Gazebo 11)
- Check `GZ_SIM_RESOURCE_PATH` includes package paths
- Verify world file exists: `src/simulation/arm_gazebo/worlds/lab.sdf`

**Robot model not appearing:**
- Check `robot_state_publisher` is running
- Verify URDF generates without errors: `xacro src/robot_description/arm_description/urdf/arm.urdf.xacro`
- Ensure mesh files exist in `arm_description/meshes/`

## File Naming Conventions

- Launch files: `*.launch.py` (Python launch files only)
- Config files: `*.yaml` (YAML configuration)
- URDF/Xacro: `*.xacro` or `*.urdf.xacro`
- Meshes: `*.STL` (uppercase extension)
- Python scripts: snake_case with `.py` extension
