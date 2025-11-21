# LDR Humanoid Arm System - Quick Reference Guide

**Quick reference for commands, launch strategies, and troubleshooting**

---

## Table of Contents

1. [System Architecture (Visual)](#1-system-architecture-visual)
2. [Essential Commands](#2-essential-commands)
3. [Launch Strategies](#3-launch-strategies)
4. [Package Reference](#4-package-reference)
5. [Debugging Commands](#5-debugging-commands)
6. [File Locations](#6-file-locations)
7. [Common Issues](#7-common-issues)
8. [Robot Specifications](#8-robot-specifications)

---

## 1. System Architecture (Visual)

### System Layers

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          USER INTERFACE LAYER                               │
│   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐                   │
│   │  RViz2  │   │   GUI   │   │Keyboard │   │Joystick │                   │
│   └────┬────┘   └────┬────┘   └────┬────┘   └────┬────┘                   │
└────────┼─────────────┼─────────────┼─────────────┼────────────────────────┘
         │             │             │             │
┌────────┼─────────────┼─────────────┼─────────────┼────────────────────────┐
│        │        APPLICATION / PLANNING LAYER      │                        │
│   ┌────▼───┐   ┌────▼────┐   ┌────▼────┐   ┌────▼────┐                   │
│   │MoveIt2 │   │ Motion  │   │ Teleop  │   │Perception│                   │
│   │Planning│   │ Planner │   │  Nodes  │   │ Pipeline │                   │
│   └────┬───┘   └────┬────┘   └────┬────┘   └────┬─────┘                   │
└────────┼────────────┼─────────────┼─────────────┼────────────────────────┘
         │            │             │             │
┌────────┼────────────┼─────────────┼─────────────┼────────────────────────┐
│        │                 CONTROL LAYER           │                        │
│   ┌────▼──────────────────────────────────────┐ │                        │
│   │       ros2_control Framework              │ │                        │
│   │  ┌────────────────────────────────────┐   │ │                        │
│   │  │  Controller Manager                │   │ │                        │
│   │  │  ┌──────────────────────────────┐  │   │ │                        │
│   │  │  │ arm_controller (6 DOF)       │  │   │ │                        │
│   │  │  │ hand_controller (gripper)    │  │   │ │                        │
│   │  │  │ joint_state_broadcaster      │  │   │ │                        │
│   │  │  └──────────────────────────────┘  │   │ │                        │
│   │  └────────────────────────────────────┘   │ │                        │
│   └────────────────┬───────────────────────────┘ │                        │
└────────────────────┼─────────────────────────────┼────────────────────────┘
                     │                             │
┌────────────────────┼─────────────────────────────┼────────────────────────┐
│          HARDWARE ABSTRACTION LAYER              │                        │
│   ┌────────────────▼────┐         ┌──────────────▼─────┐                  │
│   │ GazeboSimPlugin     │   OR    │ Hardware Interface │                  │
│   │ (Simulation)        │         │ (Real Robot)       │                  │
│   └────────────┬────────┘         └──────────┬─────────┘                  │
└────────────────┼───────────────────────────────┼────────────────────────────┘
                 │                               │
┌────────────────▼───────────────────────────────▼────────────────────────────┐
│                      PHYSICAL/SIMULATION LAYER                              │
│   ┌────────────────────┐             ┌─────────────────────┐               │
│   │ Gazebo Harmonic    │     OR      │  Physical Robot     │               │
│   │ (gz_sim)           │             │  (Motors/Sensors)   │               │
│   └────────────────────┘             └─────────────────────┘               │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Dual Control Paths

```
Path 1: Simple Control              Path 2: MoveIt Control
┌─────────────────────┐              ┌──────────────────────┐
│  motion_planner.py  │              │    moveit_py API     │
│                     │              │                      │
│ • Direct control    │              │ • IK solving         │
│ • Fast execution    │              │ • Collision avoid    │
│ • Known trajectories│              │ • Cartesian planning │
└──────────┬──────────┘              └──────────┬───────────┘
           │                                    │
           └───────────────┬────────────────────┘
                           │
                ┌──────────▼─────────────┐
                │ arm_controller         │
                │ /follow_joint_trajectory│
                └────────────────────────┘
```

---

## 2. Essential Commands

### Build & Source

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select arm_control

# Source workspace
source install/setup.bash
```

### Launch System (GUI - RECOMMENDED)

```bash
ros2 run arm_gui_tools full_system_launcher.py
```

**GUI provides:**
- World selection dropdown
- Full system launch (headless Gazebo + controllers)
- Individual component control (Gazebo GUI, RViz, MoveIt, OctoMap, etc.)
- Start/stop buttons with status monitoring

### Launch System (Command-Line - Alternative)

```bash
# Full system with default world
ros2 launch arm_system_bringup full_system.launch.py

# Full system with specific world
ros2 launch arm_system_bringup full_system.launch.py \
  simulation_world:=/path/to/world.sdf
```

### Launch System (Legacy All-in-One)

```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

### Launch with Perception

```bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

### Run Examples

```bash
# Example motion sequence
ros2 run arm_control example.py

# Simple test
ros2 run arm_control test_simple.py
```

### Check Status

```bash
# List active controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check controller manager status
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
```

---

## 3. Launch Strategies

### Strategy 1: GUI Launcher (PRIMARY - RECOMMENDED)

**Advantages:** User-friendly, visual, flexible, resource-efficient
**Use when:** All use cases, especially beginners and demos

**Command:**
```bash
ros2 run arm_gui_tools full_system_launcher.py
```

**What it does:**
1. Opens PyQt5 GUI with component controls
2. Select simulation world from dropdown (lab.sdf, obstacle_course.sdf, etc.)
3. Click "Start" for full system → headless Gazebo + controllers
4. Optionally start individual components via buttons:
   - Gazebo GUI viewer (`gz sim -g`)
   - RViz visualization
   - MoveIt planning interface
   - OctoMap server
   - Object detection (YOLO)
   - Image viewer (camera feed)
5. Each component has Start/Stop buttons with status display

**Benefits:**
- **Headless-first**: Gazebo runs headless, GUI on demand (saves resources)
- **Component independence**: Start/stop individually without affecting others
- **Visual feedback**: Clear PID and status for each component
- **Simplified workflow**: No need to manage multiple terminals
- **Process isolation**: Each component in separate process group

---

### Strategy 2: Command-Line Full System (Alternative)

**Advantages:** Single command, scriptable, headless-friendly
**Use when:** Scripting, automation, headless servers

**Command:**
```bash
ros2 launch arm_system_bringup full_system.launch.py

# With specific world
ros2 launch arm_system_bringup full_system.launch.py \
  simulation_world:=/path/to/world.sdf
```

**What it includes:**
- Headless Gazebo simulation (no GUI)
- Robot controllers (joint_state_broadcaster, arm_controller, hand_controller)
- Optional planner (commented out by default)

---

### Strategy 3: Legacy All-in-One (Reference)

**Advantages:** Single command with MoveIt + RViz
**Use when:** Legacy workflows, need everything at once

**Command:**
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

**Timeline:**
- t=0s: Gazebo world + robot spawn
- t=4s: joint_state_broadcaster
- t=5s: arm_controller
- t=6s: move_group (MoveIt)
- t=8s: RViz2
- t=10s: System ready

**Disadvantages:**
- Heavy (launches all GUIs)
- Less flexible than GUI launcher

---

### Strategy 4: Individual Components (Advanced Debugging)

**Advantages:** Maximum control, component-level debugging
**Use when:** Advanced debugging, custom setups

```bash
# Terminal 1: Gazebo world only
ros2 launch arm_gazebo arm_world.launch.py

# Terminal 2 (wait 5s): Robot + controllers
ros2 launch arm_gazebo spawn_arm.launch.py

# Terminal 3 (wait 15s): MoveIt planning
ros2 launch arm_moveit_config move_group.launch.py

# Terminal 4: RViz
ros2 launch arm_moveit_config moveit_rviz.launch.py
```

---

## 4. Package Reference

### Core Packages (ACTIVE)

| Package | Purpose | Key Features |
|---------|---------|--------------|
| **arm_description** | Robot URDF/Xacro model | 6 DOF arm + 2 DOF gripper, RGBD camera |
| **arm_control** | Simple motion control | Direct trajectory API, fast execution |
| **arm_moveit_config** | MoveIt2 planning | IK, collision avoidance, 4 planners |
| **arm_gazebo** | Gazebo Harmonic simulation | Headless mode, 4+ world files |
| **arm_perception** | 3D perception | Point cloud, YOLO, Kalman tracking |
| **arm_system_bringup** | System integration | Full system launch, OctoMap |
| **arm_gui_tools** | PyQt5 GUI tools | **PRIMARY LAUNCHER**, component control |
| **arm_demos** | Demo applications | Example use cases |
| **arm_teleop** | Teleoperation | Keyboard, joystick, GUI control |

### Package Organization

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

---

## 5. Debugging Commands

### Controller Debugging

```bash
# List controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# hand_controller[gripper_action_controller/GripperActionController] active

# List hardware interfaces
ros2 control list_hardware_interfaces

# Detailed controller information
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
```

### Topic Debugging

```bash
# List all topics
ros2 topic list

# Check publishing rate (should be ~50 Hz)
ros2 topic hz /joint_states

# Check bandwidth usage
ros2 topic bw /camera/depth/points

# Topic information
ros2 topic info /joint_states

# Print messages
ros2 topic echo /joint_states
```

### TF Debugging

```bash
# Check specific transform
ros2 run tf2_ros tf2_echo map base_link

# Generate TF tree PDF
ros2 run tf2_tools view_frames
evince frames.pdf
```

### URDF Debugging

```bash
# Validate URDF
xacro src/robot_description/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf

# Generate link diagram
urdf_to_graphiz /tmp/arm.urdf
```

### Gazebo Debugging

```bash
# List Gazebo topics
gz topic -l

# List models in simulation
gz model -l

# Model information
gz model -m arm -i
```

### Node Debugging

```bash
# List active nodes
ros2 node list

# Node details
ros2 node info /move_group

# Visualize node graph
ros2 run rqt_graph rqt_graph
```

### Performance Debugging

```bash
# Publishing rate
ros2 topic hz <topic>

# Bandwidth usage
ros2 topic bw <topic>

# CPU/memory usage
htop

# GPU usage (if using YOLO)
nvidia-smi
```

---

## 6. File Locations

### Key Configuration Files

**Controller configuration:**
- `src/control/arm_control/config/controllers.yaml`
- `src/simulation/arm_gazebo/config/controllers.yaml`

**MoveIt config:**
- `src/planning/arm_moveit_config/config/kinematics.yaml` - IK solver
- `src/planning/arm_moveit_config/config/joint_limits.yaml` - Limits
- `src/planning/arm_moveit_config/config/arm_description.srdf` - Semantic model
- `src/planning/arm_moveit_config/config/moveit_controllers.yaml`

**Perception config:**
- `src/perception/arm_perception/config/perception.yaml`
- `src/perception/arm_perception/config/object_recognition.yaml`

**OctoMap config:**
- `src/bringup/arm_system_bringup/config/octomap_server.yaml`

### Key Launch Files

**Complete system:**
- `src/bringup/arm_system_bringup/launch/moveit_gazebo.launch.py`
- `src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py`
- `src/bringup/arm_system_bringup/launch/full_system.launch.py` (RECOMMENDED)

**Modular launches:**
- `src/planning/arm_moveit_config/launch/demo.launch.py`

**Simulation:**
- `src/simulation/arm_gazebo/launch/arm_world.launch.py`
- `src/simulation/arm_gazebo/launch/spawn_arm.launch.py`

**Perception:**
- `src/perception/arm_perception/launch/perception.launch.py`

### Key Python Scripts

**Control:**
- `src/control/arm_control/scripts/motion_planner.py` - MotionPlanner class
- `src/control/arm_control/scripts/example.py` - Example usage
- `src/control/arm_control/scripts/test_simple.py` - Basic tests

**Perception:**
- `src/perception/arm_perception/scripts/perception_node.py`
- `src/perception/arm_perception/scripts/object_recognition_node.py`
- `src/perception/arm_perception/scripts/dynamic_object_tracker.py`

**Planning:**
- `src/planning/arm_moveit_config/nodes/planner_node.py`
- `src/planning/arm_moveit_config/nodes/gazebo_shape_sync.py`

### Robot Model Files

**Main URDF:**
- `src/robot_description/arm_description/urdf/arm.urdf.xacro`

**URDF components:**
- `src/robot_description/arm_description/urdf/arm_gazebo.xacro` - Sim variant
- `src/robot_description/arm_description/urdf/arm_real.xacro` - Real variant
- `src/robot_description/arm_description/urdf/links/arm_links.xacro`
- `src/robot_description/arm_description/urdf/joints/arm_joints.xacro`
- `src/robot_description/arm_description/urdf/macros/ros2_control.xacro`
- `src/robot_description/arm_description/urdf/macros/depth_camera.xacro`

**Meshes:**
- `src/robot_description/arm_description/meshes/visual/` - 40+ STL files
- `src/robot_description/arm_description/meshes/collision/` - 40+ STL files

### Simulation Worlds

- `src/simulation/arm_gazebo/worlds/lab.sdf`
- `src/simulation/arm_gazebo/worlds/lab-ldr.sdf`
- `src/simulation/arm_gazebo/worlds/obstacle_course.sdf`
- `src/simulation/arm_gazebo/worlds/whole_arm_workspace.sdf`

---

## 7. Common Issues

### Controllers not spawning

**Symptom:** "Failed to load controller" or "Controller manager not available"

**Solution:**
1. Check Gazebo is fully initialized (wait 5+ seconds)
2. Verify controller_manager is running:
   ```bash
   ros2 control list_controllers
   ```
3. Check ros2_control plugin loaded in Gazebo logs
4. Ensure proper timing in launch files (use TimerAction)

---

### MoveIt planning fails

**Symptom:** "No plans found" or "IK solver failed"

**Solution:**
1. Verify controllers are active:
   ```bash
   ros2 control list_controllers
   ```
2. Check joint limits in `joint_limits.yaml`
3. Increase IK solver timeout in `kinematics.yaml`
4. Check planning scene for collision objects blocking path
5. Try different goal positions

---

### Robot model not appearing

**Symptom:** Empty RViz or no robot in Gazebo

**Solution:**
1. Check `robot_state_publisher` is running:
   ```bash
   ros2 node list | grep robot_state_publisher
   ```
2. Verify URDF generates without errors:
   ```bash
   xacro src/robot_description/arm_description/urdf/arm.urdf.xacro
   ```
3. Check mesh files exist:
   ```bash
   ls src/robot_description/arm_description/meshes/visual/
   ```
4. In RViz, add RobotModel display and set Fixed Frame to "base_link"

---

### Gazebo simulation not starting

**Symptom:** `gz sim` not found or world doesn't load

**Solution:**
1. Ensure Gazebo Harmonic installed (not legacy Gazebo 11):
   ```bash
   gz sim --version
   ```
2. Check `GZ_SIM_RESOURCE_PATH` includes package paths
3. Verify world file exists:
   ```bash
   ls src/simulation/arm_gazebo/worlds/lab.sdf
   ```
4. Check Gazebo logs for errors:
   ```bash
   ~/.gz/sim/server.log
   ```

---

### /joint_states not published

**Symptom:** No joint state messages

**Solution:**
1. Check `joint_state_broadcaster` is active:
   ```bash
   ros2 control list_controllers
   ```
2. Verify controller spawned with proper delay (4s after Gazebo)
3. Check topic:
   ```bash
   ros2 topic hz /joint_states
   ```
4. Restart `joint_state_broadcaster`:
   ```bash
   ros2 control switch_controllers --deactivate joint_state_broadcaster
   ros2 control switch_controllers --activate joint_state_broadcaster
   ```

---

### Perception not detecting objects

**Symptom:** Empty `/detected_objects` topic

**Solution:**
1. Check point cloud is published:
   ```bash
   ros2 topic hz /camera/depth/points
   ```
2. Verify `perception_node` is running:
   ```bash
   ros2 node list | grep perception
   ```
3. Check perception parameters in `perception.yaml`
4. Visualize point cloud in RViz to verify camera working
5. Adjust clustering parameters if objects too small/large

---

### Build errors

**Symptom:** Compilation failures

**Solution:**
1. Clean build:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```
2. Check dependencies installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build with verbose output:
   ```bash
   colcon build --event-handlers console_direct+
   ```
4. Build single package to isolate issue:
   ```bash
   colcon build --packages-select <package_name>
   ```

---

## 8. Robot Specifications

### Kinematic Structure

**Total DOF:** 8 actuated joints
- **6 DOF arm** (shoulder pitch/roll/yaw, elbow, wrist pitch/roll)
- **2 DOF gripper** (parallel jaw, 1 mimic joint)

**Links:** 14 total
- base_link
- Shoulder: actuator, flange, pitch, roll, yaw (5 links)
- Elbow: actuator, flange, elbow (3 links)
- Wrist: actuator, flange (2 links)
- Hand: hand, palm, left/right finger (4 links)

### Arm Joint Specifications

| Joint Name | Type | Limits (rad) | Torque | Velocity |
|------------|------|--------------|--------|----------|
| `left_shoulder_pitch_rs04` | revolute | [-3.14, 3.14] | 120 Nm | 3.0 rad/s |
| `left_shoulder_roll_rs04` | revolute | [-3.5, 0.6] | 120 Nm | 3.0 rad/s |
| `left_shoulder_yaw_rs03` | revolute | [-3.14, 3.14] | 60 Nm | 1.6 rad/s |
| `left_elbow_rs03` | revolute | [-1.3, 1.9] | 60 Nm | 1.6 rad/s |
| `left_wrist_rs02` | revolute | [-3.14, 3.14] | 17 Nm | 0.8 rad/s |
| `left_hand_rs02` | revolute | [-3.14, 3.14] | 17 Nm | 0.8 rad/s |

### Gripper Joint Specifications

| Joint Name | Type | Limits (rad) | Notes |
|------------|------|--------------|-------|
| `left_palm_right_finger` | revolute | [0, 0.8] | Main control |
| `left_palm_left_finger` | revolute | [0, 0.8] | Mimic (automatic) |

### Sensor Specifications

**RGBD Camera:**
- **Mount:** base_link (1.75m height, 15° downward tilt)
- **Type:** Gazebo depth camera plugin
- **Resolution:** 640x480 pixels
- **FOV:** 60° horizontal, 45° vertical
- **Range:** 0.3m - 10m
- **Frame rate:** 30 Hz

**Topics:**
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/depth/points` (sensor_msgs/PointCloud2)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo)

### Workspace

- **Estimated reach:** ~1.2m radius
- **Operating height:** 0.3m - 2.0m above base
- **Gripper capacity:** ~5 kg payload (estimated)

### ROS 2 Interfaces

**Published Topics:**
- `/joint_states` (sensor_msgs/JointState) @ 50 Hz
- `/robot_description` (std_msgs/String) @ on startup
- `/tf`, `/tf_static` (tf2_msgs/TFMessage) @ 50 Hz
- `/camera/color/image_raw` (sensor_msgs/Image) @ 30 Hz
- `/camera/depth/image_raw` (sensor_msgs/Image) @ 30 Hz
- `/camera/depth/points` (sensor_msgs/PointCloud2) @ 30 Hz
- `/planning_scene` (moveit_msgs/PlanningScene) @ on change
- `/detected_objects` (visualization_msgs/MarkerArray) @ 1 Hz

**Action Servers:**
- `/arm_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory)
- `/hand_controller/gripper_cmd` (control_msgs/GripperCommand)

### System Requirements

**Minimum:**
- CPU: 4 cores
- RAM: 8 GB
- OS: Ubuntu 24.04 (Noble)
- ROS 2: Jazzy

**Recommended:**
- CPU: 8+ cores
- RAM: 16 GB
- GPU: Optional (for YOLO perception)
- Storage: SSD

---

## Usage Examples

### Example 1: Simple Motion Control

```python
from motion_planner import MotionPlanner

planner = MotionPlanner()
planner.home()                                    # Go to home position
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3, 0.0]) # Move to joint positions
planner.ready()                                   # Go to ready position
```

### Example 2: MoveIt Planning

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="planner")
arm = moveit.get_planning_component("arm")

# Plan to named pose
arm.set_goal(pose_name="home")
plan = arm.plan()
if plan:
    arm.execute()
```

### Example 3: Check System Status

```bash
# List controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## Quick Troubleshooting Flowchart

### System Won't Start
- **Gazebo won't launch** → Check: `gz sim --version` (should be Harmonic), check `~/.gz/sim/server.log`
- **Controllers won't spawn** → Wait 5+ seconds after Gazebo starts, check: `ros2 control list_controllers`
- **Robot not visible** → Check: `robot_state_publisher` running, URDF compiles, mesh files exist
- **MoveIt won't start** → Check: Controllers active, `/joint_states` being published

### Planning Fails
- **"No plans found"** → Try different goal, increase planning time, check for collisions
- **"IK solver failed"** → Check goal is within workspace, increase IK timeout in `kinematics.yaml`
- **"Planning scene empty"** → Check `robot_state_publisher` running, check `/planning_scene` topic

### Perception Not Working
- **No point cloud** → Check camera in Gazebo, check `/camera/depth/points` topic
- **No objects detected** → Check point cloud in RViz, adjust clustering parameters, verify objects in FOV
- **Objects not in planning scene** → Check `planning_scene_updater` node running, check `/planning_scene` topic

---

**For complete documentation, see:**
- [ARCHITECTURE.md](ARCHITECTURE.md) - Complete architecture specification
- [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) - Visual diagrams (Mermaid)
- [README.md](README.md) - Documentation index
- [../CLAUDE.md](../CLAUDE.md) - Project overview

**Repository:** https://github.com/widemic/ldr-humanoid-arm-system

**Last Updated:** 2025-11-21
