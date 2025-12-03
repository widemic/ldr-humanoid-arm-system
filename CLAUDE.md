# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Documentation](#documentation)
3. [Quick Start](#quick-start)
4. [Robot Specifications](#robot-specifications)
5. [Build System](#build-system)
6. [Running the System](#running-the-system)
7. [Testing](#testing)
8. [Package Architecture](#package-architecture)
9. [Control Architecture](#control-architecture)
10. [Development Guidelines](#development-guidelines)
11. [Common Issues](#common-issues)

---

## Project Overview

This is a **ROS 2 Jazzy** robotics control system for a **6-DOF humanoid arm with gripper**. The system provides both simple direct control and advanced MoveIt2 motion planning capabilities, with full Gazebo Harmonic simulation support and 3D perception.

**Tech Stack:**
- ROS 2 Jazzy on Ubuntu 24.04 (Noble)
- Gazebo Harmonic for physics simulation
- MoveIt2 for motion planning
- ros2_control framework for controller management
- Python 3 + C++ with ament_cmake build system
- PCL + OctoMap for 3D perception
- YOLO for object recognition

**Repository:** https://github.com/widemic/ldr-humanoid-arm-system

---

## Documentation

**Complete architecture documentation is available in the [docs/](docs/) folder:**

- **[docs/README.md](docs/README.md)** - Documentation index and navigation guide
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - Complete system architecture (70+ pages)
- **[docs/ARCHITECTURE_DIAGRAMS.md](docs/ARCHITECTURE_DIAGRAMS.md)** - 15 visual diagrams (Mermaid)
- **[docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)** - Terminal-friendly quick reference

**When to use which document:**
- **Quick commands** → This file (CLAUDE.md) or [QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)
- **Understanding architecture** → [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- **Visual overview** → [docs/ARCHITECTURE_DIAGRAMS.md](docs/ARCHITECTURE_DIAGRAMS.md)
- **Finding specific info** → [docs/README.md](docs/README.md) navigation guide

---

## Quick Start

```bash
# 1. Build workspace
colcon build
source install/setup.bash

# 2. Launch system using GUI (RECOMMENDED)
ros2 run arm_gui_tools full_system_launcher.py

# The GUI provides:
# - Full system launch with world selection
# - Individual component control (Gazebo, RViz, MoveIt, OctoMap, etc.)
# - Start/stop buttons for each component
# - Visual status monitoring

# 3. Run example
ros2 run arm_control example.py
```

**Alternative:** See [Running the System](#running-the-system) for command-line launch options.

---

## Robot Specifications

**8 Actuated Joints:**

**Arm (6 DOF):**
1. `left_shoulder_pitch_rs04` - Shoulder pitch ([-3.14, 3.14] rad, 120 Nm, 3.0 rad/s)
2. `left_shoulder_roll_rs04` - Shoulder roll ([-3.5, 0.6] rad, 120 Nm, 3.0 rad/s)
3. `left_shoulder_yaw_rs03` - Shoulder yaw ([-3.14, 3.14] rad, 60 Nm, 1.6 rad/s)
4. `left_elbow_rs03` - Elbow ([-1.3, 1.9] rad, 60 Nm, 1.6 rad/s)
5. `left_wrist_rs02` - Wrist pitch ([-3.14, 3.14] rad, 17 Nm, 0.8 rad/s)
6. `left_hand_rs02` - Wrist roll ([-3.14, 3.14] rad, 17 Nm, 0.8 rad/s)

**Gripper (2 DOF):**
7. `left_palm_right_finger` - Right finger ([0, 0.8] rad, main control)
8. `left_palm_left_finger` - Left finger ([0, 0.8] rad, mimic joint)

**Sensors:**
- RGBD Camera (640x480 @ 30Hz, mounted on base_link)
  - Color image: `/camera/color/image_raw`
  - Depth image: `/camera/depth/image_raw`
  - Point cloud: `/camera/depth/points`

**Links:** 14 total (base, shoulder, elbow, wrist, hand, fingers)

**For complete specifications, see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) Section 6 or [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md) Section 8.**

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

### GUI Launcher (RECOMMENDED)

The primary way to launch the system is using the GUI tool:

```bash
ros2 run arm_gui_tools full_system_launcher.py
```

**Features:**
- **World Selection**: Dropdown menu to choose simulation world (lab.sdf, obstacle_course.sdf, etc.)
- **Full System Launch**: Single button to start complete system (headless sim + controllers + planner)
- **Individual Components**: Start/stop individual components:
  - Gazebo GUI viewer (`gz sim -g`)
  - RViz visualization
  - MoveIt demo (planning interface)
  - OctoMap server
  - Object detection (YOLO)
  - Image viewer (camera feed)
- **Status Monitoring**: Real-time status display for each component
- **Process Management**: Automatic cleanup of finished processes

**What the full system includes:**
- `ros2 launch arm_system_bringup full_system.launch.py simulation_world:=<selected_world>`
  - Headless Gazebo simulation (no GUI by default)
  - Robot controllers (joint_state_broadcaster, arm_controller, hand_controller)
  - Optional: MoveIt planner (commented out by default, enable via GUI)

**Workflow:**
1. Launch GUI tool
2. Select desired world from dropdown
3. Click "Start" for full system
4. Optionally start Gazebo GUI, RViz, or MoveIt via individual buttons
5. Use "Stop" buttons to terminate components when done

### Command-Line Launch (Alternative)

For scripting or headless environments:

```bash
# Full system with default world
ros2 launch arm_system_bringup full_system.launch.py

# Full system with specific world
ros2 launch arm_system_bringup full_system.launch.py \
  simulation_world:=/path/to/world.sdf

# Headless simulation only
ros2 launch arm_gazebo headless_sim.launch.py

# Controllers only (requires simulation running)
ros2 launch arm_control control.launch.py

# MoveIt planning only
ros2 launch arm_moveit_config planner.launch.py

# Complete system with MoveIt + RViz (legacy)
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

### Individual Component Launch (Advanced)

For debugging specific components:

```bash
# Gazebo world + robot
ros2 launch arm_gazebo arm_world.launch.py

# Spawn robot in existing world
ros2 launch arm_gazebo spawn_arm.launch.py

# Gazebo GUI viewer only
gz sim -g

# RViz with perception config
rviz2 -d $(ros2 pkg prefix arm_perception)/share/arm_perception/config/deep_camera.rviz

# MoveIt demo with RViz
ros2 launch arm_moveit_config demo.launch.py

# OctoMap server
ros2 launch arm_system_bringup octomap_server.launch.py
```

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

## MoveIt Task Constructor (MTC)

The system includes MoveIt Task Constructor for advanced task-level manipulation planning.

### Running MTC Demos

**Simple approach-retreat demo:**
```bash
# Launch MTC environment with simple demo
ros2 launch arm_demos mtc_demo.launch.py demo:=simple

# With Gazebo simulation
ros2 launch arm_demos mtc_demo.launch.py demo:=simple use_sim:=true
```

**Pick-and-place demo (future - requires gripper):**
```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=pick_place
```

**Launch MTC environment only (for custom scripts):**
```bash
ros2 launch arm_demos mtc_demo.launch.py demo:=none
```

### MTC Configuration Files

**[src/planning/arm_moveit_config/config/mtc_capabilities.yaml](src/planning/arm_moveit_config/config/mtc_capabilities.yaml)** - MTC capabilities for move_group
- ExecuteTaskSolutionCapability for task execution
- Planning and execution parameters
- Cartesian path configuration

**[src/planning/arm_moveit_config/config/mtc_solvers.yaml](src/planning/arm_moveit_config/config/mtc_solvers.yaml)** - Stage solver configurations
- Cartesian planner settings
- Joint interpolation parameters
- OMPL and Pilz planner configurations
- Workspace bounds

### MTC Demo Scripts

**[src/applications/arm_demos/scripts/mtc_simple_demo.py](src/applications/arm_demos/scripts/mtc_simple_demo.py)** - Basic MTC demonstration
- Shows fundamental MTC concepts
- Stages: CurrentState → MoveTo(home) → MoveRelative(approach) → MoveRelative(retreat) → MoveTo(home)
- Good starting point for learning MTC

**[src/applications/arm_demos/scripts/mtc_pick_place_demo.py](src/applications/arm_demos/scripts/mtc_pick_place_demo.py)** - Advanced pick-and-place
- Complete pick-and-place pipeline
- Includes grasp generation and object attachment
- Will be fully functional when gripper is integrated

### MTC Stage Types

Common stage types available:
- **CurrentState** - Initialize task from current robot state
- **MoveTo** - Move to named pose or joint configuration
- **MoveRelative** - Move relative to current pose (Cartesian or joint space)
- **Connect** - Connect two task states with motion planning
- **GenerateGraspPose** - Generate candidate grasp poses for objects
- **GeneratePlacePose** - Generate candidate place poses
- **ComputeIK** - Compute inverse kinematics for poses
- **ModifyPlanningScene** - Attach/detach objects, allow/forbid collisions

### Creating Custom MTC Tasks

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Vector3Stamped

class MyCustomTask(Node):
    def __init__(self):
        super().__init__('my_task')
        self.task = core.Task("my_custom_task")
        self.task.loadRobotModel(self.get_logger())

        # Set properties
        self.task.setProperty("group", "arm")
        self.task.setProperty("ik_frame", "wrist_roll_link")

        # Create planners
        cartesian = core.CartesianPath()
        cartesian.setStepSize(0.01)

        sampling = core.PipelinePlanner()

        # Build task stages
        current = stages.CurrentState("current")
        self.task.add(current)

        # Add your custom stages here...

    def plan(self):
        self.task.plan()
        return self.task.numSolutions() > 0

    def execute(self):
        if self.task.numSolutions() > 0:
            self.task.execute(self.task.solutions()[0])
```

## Package Architecture

### Package Organization

**25 ROS 2 packages** organized into **9 categories**:

```
src/
├── robot_description/      # Robot models (4 packages)
│   ├── arm_description         [ACTIVE]
│   └── gripper_*, hand_*, dual_arm_* [PLACEHOLDER]
├── control/                # Control interfaces (5 packages)
│   ├── arm_control             [ACTIVE]
│   └── teleop/arm_teleop       [PARTIAL]
├── planning/               # Motion planning (5 packages)
│   ├── arm_moveit_config       [ACTIVE]
│   └── arm_mtc                 [MINIMAL]
├── simulation/             # Gazebo simulation (1 package)
│   └── arm_gazebo              [ACTIVE]
├── perception/             # 3D perception (2 packages)
│   └── arm_perception          [ACTIVE]
├── hardware_interface/     # Real hardware (3 packages) [PLACEHOLDER]
├── bringup/                # System integration (1 package)
│   └── arm_system_bringup      [ACTIVE]
├── applications/           # Demo applications (3 packages)
│   └── arm_demos               [PARTIAL]
└── tools/                  # Development tools (2 packages)
    └── arm_gui_tools           [PARTIAL]
```

### Core Active Packages

**[arm_description](src/robot_description/arm_description/)** - Robot URDF/Xacro model
- Main: `urdf/arm.urdf.xacro` (switches between sim/real via `use_sim` argument)
- Links: `urdf/links/arm_links.xacro` (14 links with inertia)
- Joints: `urdf/joints/arm_joints.xacro` (8 actuated joints)
- Control: `urdf/macros/ros2_control.xacro` (ros2_control interface)
- Camera: `urdf/macros/depth_camera.xacro` (RGBD sensor)
- Meshes: 80+ STL files (visual + collision)

**[arm_control](src/control/arm_control/)** - Simple control interface
- Simple motion planner API: `scripts/motion_planner.py`
- Controllers config: `config/controllers.yaml` (arm_controller, hand_controller, joint_state_broadcaster)
- Launch: `launch/sim.launch.py` (full simulation), `launch/control.launch.py` (controllers only)

**[arm_moveit_config](src/planning/arm_moveit_config/)** - MoveIt2 planning
- SRDF: `config/arm_description.srdf` (planning groups "arm" + "hand", predefined poses, 298 collision pairs)
- Kinematics: `config/kinematics.yaml` (KDL solver, 50ms timeout)
- Pipelines: OMPL, Pilz, CHOMP, STOMP planners
- 3D sensors: `config/sensors_3d.yaml` (OctoMap integration)
- Launch: `launch/demo.launch.py` (MoveIt + RViz), `launch/move_group.launch.py` (planning only)

**[arm_gazebo](src/simulation/arm_gazebo/)** - Gazebo Harmonic simulation
- Worlds: `worlds/lab.sdf`, `worlds/obstacle_course.sdf`, etc.
- Models: 15+ Gazebo models (tables, chairs, obstacles)
- Launch: `launch/arm_world.launch.py` (world + robot), `launch/spawn_arm.launch.py` (robot + controllers)
- Config: `config/controllers.yaml`, `config/bridge.yaml` (ROS-Gazebo bridge)

**[arm_perception](src/perception/arm_perception/)** - 3D perception pipeline
- Point cloud processing: `scripts/perception_node.py` (plane removal, clustering, shape detection)
- Object recognition: `scripts/object_recognition_node.py` (YOLO detection)
- Object tracking: `scripts/dynamic_object_tracker.py` (Kalman filtering)
- MoveIt sync: `scripts/planning_scene_updater.py` (collision object updates)
- Config: `config/perception.yaml`, `config/object_recognition.yaml`

**[arm_system_bringup](src/bringup/arm_system_bringup/)** - System integration
- Complete demo: `launch/moveit_gazebo.launch.py` (Gazebo + MoveIt + RViz)

**[arm_demos](src/applications/arm_demos/)** - Demo applications and examples
- MTC demos: `scripts/mtc_simple_demo.py`, `scripts/mtc_pick_place_demo.py`
- Launch: `launch/mtc_demo.launch.py` (MTC environment + demos)

### Placeholder Packages (Not Yet Implemented)
- Hardware interfaces (`arm_hardware`)
- Gripper/hand packages (`gripper_*`, `hand_*`)
- Dual-arm configurations (`dual_arm_*`)
- GUI/diagnostic tools
- Launch: `launch/full_system.launch.py` (headless sim + controllers, RECOMMENDED)
- Legacy: `launch/moveit_gazebo.launch.py` (Gazebo + MoveIt + RViz)
- With perception: `launch/moveit_gazebo_with_octomap.launch.py` (+ perception pipeline)
- Components: `launch/octomap_server.launch.py`, `launch/headless.launch.py`
- Config: `config/octomap_server.yaml`, `config/gazebo_bridge.yaml`

**[arm_gui_tools](src/tools/arm_gui_tools/)** - PyQt5 GUI tools (RECOMMENDED)
- System launcher: `src/arm_gui_tools/full_system_launcher.py` (PRIMARY LAUNCH METHOD)
  - Full system launch with world selection
  - Individual component control (Gazebo, RViz, MoveIt, OctoMap, object detection)
  - Real-time status monitoring and process management
- Joint monitor: `src/arm_gui_tools/joint_monitor.py` (joint state visualization)
- UI files: `ui/full_system_launcher.ui`, `ui/images/`

**For complete package details and dependencies, see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) Section 2.**

## Control Architecture

### Two Control Paths

**Path 1: Simple Motion Planner (arm_control)**
```python
from motion_planner import MotionPlanner

planner = MotionPlanner()
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3, 0.0])  # 6 joint positions (arm)
planner.home()  # Move to home position
planner.ready() # Move to ready position
```
- Direct trajectory control, no MoveIt overhead
- Fast and lightweight (microseconds)
- Good for known trajectories
- Simple API for basic motions

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
- Collision avoidance (OctoMap integration)
- Inverse kinematics (KDL solver)
- Cartesian planning
- Interactive RViz visualization
- Multiple planning pipelines (OMPL, Pilz, CHOMP, STOMP)
- Predefined poses (home, ready, open, close)

**Both paths share the same controller interface:** `/arm_controller/follow_joint_trajectory` action

**Path 3: MoveIt Task Constructor (MTC)**
```python
from moveit.task_constructor import core, stages

task = core.Task("my_task")
task.loadRobotModel()

# Add stages: CurrentState → MoveTo → MoveRelative → etc.
current = stages.CurrentState("current")
task.add(current)

# Plan and execute
task.plan()
task.execute(task.solutions()[0])
```
- Sequential task planning with reusable stages
- Pick-and-place automation
- Complex multi-step manipulation
- Stage-based composition (approach, grasp, lift, place, retreat)

**All paths share the same controller interface:** `arm_controller/follow_joint_trajectory` action
**See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) Section 3 for detailed control flow diagrams.**

### Controller Stack

```
User Code (Simple or MoveIt)
    ↓
/arm_controller/follow_joint_trajectory (Action)
    ↓
arm_controller (JointTrajectoryController)
    ↓
ros2_control → GazeboSimROS2ControlPlugin OR HardwareInterface
    ↓
Gazebo Physics OR Real Robot Motors
    ↓
joint_state_broadcaster → /joint_states (50 Hz)
```

**Three controllers defined in config/controllers.yaml:**
1. `joint_state_broadcaster` - Publishes joint states at 50 Hz
2. `arm_controller` - JointTrajectoryController for 6 arm joints (position/velocity)
3. `hand_controller` - GripperActionController for 2-finger gripper

## Important Configuration Files

**Controller configuration:**
- [src/control/arm_control/config/controllers.yaml](src/control/arm_control/config/controllers.yaml) - Joint state broadcaster, arm_controller, hand_controller
- [src/simulation/arm_gazebo/config/controllers.yaml](src/simulation/arm_gazebo/config/controllers.yaml) - Same as above
- [src/control/arm_control/config/actuator_specs.yaml](src/control/arm_control/config/actuator_specs.yaml) - Motor specifications

**MoveIt planning:**
- [src/planning/arm_moveit_config/config/kinematics.yaml](src/planning/arm_moveit_config/config/kinematics.yaml) - IK solver (KDL, 50ms timeout)
- [src/planning/arm_moveit_config/config/joint_limits.yaml](src/planning/arm_moveit_config/config/joint_limits.yaml) - Planning velocity/acceleration limits
- [src/planning/arm_moveit_config/config/arm_description.srdf](src/planning/arm_moveit_config/config/arm_description.srdf) - Planning groups, poses, 298 collision pairs
- [src/planning/arm_moveit_config/config/moveit_controllers.yaml](src/planning/arm_moveit_config/config/moveit_controllers.yaml) - MoveIt controller interface
- [src/planning/arm_moveit_config/config/sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml) - OctoMap point cloud config

**Perception:**
- [src/perception/arm_perception/config/perception.yaml](src/perception/arm_perception/config/perception.yaml) - Point cloud processing parameters
- [src/perception/arm_perception/config/object_recognition.yaml](src/perception/arm_perception/config/object_recognition.yaml) - YOLO configuration
- [src/bringup/arm_system_bringup/config/octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml) - OctoMap server settings

**MoveIt Task Constructor:**
- [src/planning/arm_moveit_config/config/mtc_capabilities.yaml](src/planning/arm_moveit_config/config/mtc_capabilities.yaml) - MTC capabilities and execution parameters
- [src/planning/arm_moveit_config/config/mtc_solvers.yaml](src/planning/arm_moveit_config/config/mtc_solvers.yaml) - Stage solver configurations (cartesian, sampling, interpolation)

**Robot model:**
- [src/robot_description/arm_description/urdf/arm.urdf.xacro](src/robot_description/arm_description/urdf/arm.urdf.xacro) - Main entry (use_sim:=true/false)
- [src/robot_description/arm_description/urdf/macros/ros2_control.xacro](src/robot_description/arm_description/urdf/macros/ros2_control.xacro) - Control interface
- [src/robot_description/arm_description/urdf/macros/depth_camera.xacro](src/robot_description/arm_description/urdf/macros/depth_camera.xacro) - RGBD camera

**Simulation:**
- [src/simulation/arm_gazebo/config/bridge.yaml](src/simulation/arm_gazebo/config/bridge.yaml) - ROS-Gazebo topic bridge
- [src/simulation/arm_gazebo/worlds/lab.sdf](src/simulation/arm_gazebo/worlds/lab.sdf) - Main simulation world

**For complete file reference, see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) Appendix A.**

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
- Planning group "arm" includes 6 arm joints
- Planning group "hand" includes 2 gripper joints
- KDL kinematics solver is default (TRAC-IK available as alternative)
- SRDF defines 298 disabled collision pairs for performance
- Four planning pipelines available: OMPL, Pilz, CHOMP, STOMP
- OctoMap integration for 3D collision detection
- Test planning with RViz interactive markers before coding

### Working with MoveIt Task Constructor
- MTC tasks are composed of sequential stages
- Each stage generates or forwards robot states
- Use SerialContainer for sequential execution, ParallelContainer for alternatives
- Stages can be configured with properties (group, IK frame, timeout, etc.)
- Always start with CurrentState stage to initialize from robot's current state
- Use introspection (`task.toString()`) to debug task structure
- MTC requires move_group with ExecuteTaskSolutionCapability enabled
### Working with Perception
- Point cloud processing runs at 1 Hz (configurable in perception.yaml)
- Plane segmentation uses RANSAC with 1cm threshold
- Clustering tolerance is 5cm, 200-10,000 points per cluster
- YOLO runs on detected clusters for object classification
- Kalman filtering tracks object positions and velocities
- Objects automatically added to MoveIt planning scene
- Adjust parameters in perception.yaml for different environments

### Working with Launch Files
- Use TimerAction for sequential initialization (see [moveit_gazebo.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo.launch.py))
- Always set `use_sim_time: True` for Gazebo nodes
- Gazebo Harmonic uses `gz_sim` (not `gazebo` legacy)
- Clock bridge synchronizes ROS time with Gazebo time

## Key Architectural Decisions

1. **Modular package structure** - Control, planning, simulation, perception, and description are separate for flexibility
2. **GUI-based system launcher** - PyQt5 GUI tool as primary launch method with world selection and component control
3. **Dual control paradigm** - Simple API for basic control, MoveIt for complex planning (both share same controller)
4. **Headless-first simulation** - Gazebo runs headless by default; GUI viewer launched separately on demand
5. **Simulation-first design** - Use `use_sim` flag to switch between simulation and real hardware
6. **OctoMap integration** - Real-time 3D collision avoidance from camera point cloud
7. **Component independence** - Each system component can be started/stopped individually via GUI

**For complete architectural details, see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).**

---

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
- In RViz, add RobotModel display and set Fixed Frame to "base_link"

**Perception not detecting objects:**
- Check point cloud is published: `ros2 topic hz /camera/depth/points`
- Verify perception_node is running: `ros2 node list | grep perception`
- Visualize point cloud in RViz to verify camera working
- Adjust clustering parameters in perception.yaml if objects too small/large

**Build errors:**
- Clean build: `rm -rf build/ install/ log/ && colcon build`
- Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Build with verbose output: `colcon build --event-handlers console_direct+`

**For complete troubleshooting guide, see [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md) Section 7.**

---

## Additional Resources

- **Complete Architecture** - [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- **Visual Diagrams** - [docs/ARCHITECTURE_DIAGRAMS.md](docs/ARCHITECTURE_DIAGRAMS.md)
- **Quick Reference** - [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)
- **Documentation Index** - [docs/README.md](docs/README.md)

## File Naming Conventions

- Launch files: `*.launch.py` (Python launch files only)
- Config files: `*.yaml` (YAML configuration)
- URDF/Xacro: `*.xacro` or `*.urdf.xacro`
- Meshes: `*.STL` (uppercase extension)
- Python scripts: snake_case with `.py` extension

---

**Last Updated:** 2025-11-21
**Repository:** https://github.com/widemic/ldr-humanoid-arm-system
