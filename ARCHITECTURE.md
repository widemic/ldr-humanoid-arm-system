# LDR Humanoid Arm System - Architecture Documentation

**Version:** 1.0
**Date:** 2025-11-21
**ROS 2 Version:** Jazzy (Ubuntu 24.04)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Package Architecture](#2-package-architecture)
3. [Control Architecture](#3-control-architecture)
4. [Data Flow Architecture](#4-data-flow-architecture)
5. [Launch System Architecture](#5-launch-system-architecture)
6. [Robot Model Architecture](#6-robot-model-architecture)
7. [Perception Architecture](#7-perception-architecture)
8. [Build System Architecture](#8-build-system-architecture)
9. [Deployment Architectures](#9-deployment-architectures)
10. [Future Extensions](#10-future-extensions)

---

## 1. System Overview

### 1.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    USER INTERFACE LAYER                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │  RViz2   │  │ GUI Tool │  │ Keyboard │  │ Joystick │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
└───────┼─────────────┼─────────────┼─────────────┼──────────────────┘
        │             │             │             │
┌───────┼─────────────┼─────────────┼─────────────┼──────────────────┐
│       │      APPLICATION / PLANNING LAYER        │                  │
│  ┌────▼────┐  ┌─────────────┐  ┌────────────┐  │                  │
│  │ MoveIt2 │  │   Motion    │  │  Teleop    │◄─┘                  │
│  │Planning │  │  Planner    │  │   Nodes    │                     │
│  └────┬────┘  └──────┬──────┘  └──────┬─────┘                     │
│       │              │                 │                           │
│  ┌────▼──────────────▼─────────────────▼─────┐                    │
│  │         Perception Pipeline                │                    │
│  │  ┌────────────┐ ┌─────────┐ ┌───────────┐│                    │
│  │  │  OctoMap   │ │  YOLO   │ │  Tracker  ││                    │
│  │  └────────────┘ └─────────┘ └───────────┘│                    │
│  └─────────────────┬──────────────────────────┘                    │
└────────────────────┼───────────────────────────────────────────────┘
                     │
┌────────────────────▼───────────────────────────────────────────────┐
│               CONTROL LAYER                                         │
│  ┌──────────────────────────────────────────────────────────┐     │
│  │            ros2_control Framework                         │     │
│  │  ┌────────────────┐  ┌──────────────────────────────┐   │     │
│  │  │ Controller Mgr │  │   Controllers                │   │     │
│  │  └───────┬────────┘  │  ┌────────────────────────┐  │   │     │
│  │          │           │  │ arm_controller (6DOF)  │  │   │     │
│  │          │           │  │ hand_controller (grip) │  │   │     │
│  │          │           │  │ joint_state_broadcaster│  │   │     │
│  │          │           │  └────────────────────────┘  │   │     │
│  │          │           └──────────────────────────────┘   │     │
│  └──────────┼──────────────────────────────────────────────┘     │
└─────────────┼──────────────────────────────────────────────────────┘
              │
┌─────────────▼──────────────────────────────────────────────────────┐
│         HARDWARE ABSTRACTION LAYER                                 │
│  ┌────────────────────┐           ┌─────────────────────┐         │
│  │  GazeboSimPlugin   │    OR     │  HardwareInterface  │         │
│  │  (Simulation)      │           │  (Real Robot)       │         │
│  └─────────┬──────────┘           └──────────┬──────────┘         │
└────────────┼─────────────────────────────────┼────────────────────┘
             │                                  │
┌────────────▼──────────────────────────────────▼────────────────────┐
│         PHYSICAL/SIMULATION LAYER                                  │
│  ┌────────────────────┐           ┌─────────────────────┐         │
│  │  Gazebo Harmonic   │    OR     │   Physical Robot    │         │
│  │  (gz_sim)          │           │   (Motors/Sensors)  │         │
│  └────────────────────┘           └─────────────────────┘         │
└────────────────────────────────────────────────────────────────────┘
```

### 1.2 System Components Summary

| Layer | Components | Purpose |
|-------|------------|---------|
| **User Interface** | RViz2, GUI Tools, Teleop | Visualization and user interaction |
| **Application** | MoveIt2, Motion Planner, Perception | High-level task planning and sensing |
| **Control** | ros2_control, Controllers | Real-time trajectory execution |
| **Hardware Abstraction** | Gazebo Plugin / Hardware Interface | Simulation or real hardware bridge |
| **Physical** | Gazebo / Real Robot | Actual execution environment |

---

## 2. Package Architecture

### 2.1 Package Organization

The system contains **25 ROS 2 packages** organized into **9 functional categories**:

```
ldr-humanoid-arm-system/
├── src/
│   ├── robot_description/      # Robot models (4 packages)
│   │   ├── arm_description         [ACTIVE]
│   │   ├── gripper_description     [PLACEHOLDER]
│   │   ├── hand_description        [PLACEHOLDER]
│   │   └── dual_arm_description    [PLACEHOLDER]
│   │
│   ├── control/                # Control interfaces (5 packages)
│   │   ├── arm_control             [ACTIVE]
│   │   ├── gripper_control         [PLACEHOLDER]
│   │   ├── hand_control            [PLACEHOLDER]
│   │   ├── dual_arm_control        [PLACEHOLDER]
│   │   └── teleop/
│   │       └── arm_teleop          [PARTIAL]
│   │
│   ├── planning/               # Motion planning (5 packages)
│   │   ├── arm_moveit_config       [ACTIVE]
│   │   ├── arm_gripper_moveit_config [PLACEHOLDER]
│   │   ├── arm_hand_moveit_config  [PLACEHOLDER]
│   │   ├── dual_arm_moveit_config  [PLACEHOLDER]
│   │   └── arm_mtc                 [MINIMAL]
│   │
│   ├── simulation/             # Gazebo simulation (1 package)
│   │   └── arm_gazebo              [ACTIVE]
│   │
│   ├── perception/             # 3D perception (2 packages)
│   │   ├── arm_perception          [ACTIVE]
│   │   └── perception_tests        [PLACEHOLDER]
│   │
│   ├── hardware_interface/     # Real hardware (3 packages)
│   │   ├── arm_hardware            [PLACEHOLDER]
│   │   ├── gripper_hardware        [PLACEHOLDER]
│   │   └── hand_hardware           [PLACEHOLDER]
│   │
│   ├── bringup/                # System integration (1 package)
│   │   └── arm_system_bringup      [ACTIVE]
│   │
│   ├── applications/           # Demo applications (3 packages)
│   │   ├── arm_demos               [PARTIAL]
│   │   ├── manipulation_library    [PLACEHOLDER]
│   │   └── bimanual_demos          [PLACEHOLDER]
│   │
│   └── tools/                  # Development tools (2 packages)
│       ├── arm_gui_tools           [PARTIAL]
│       └── diagnostic_tools        [PLACEHOLDER]
```

### 2.2 Package Dependency Graph

```
                    ┌─────────────────────┐
                    │  arm_description    │ ◄─── Base package
                    └──────────┬──────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   ┌─────────────┐      ┌─────────────┐     ┌─────────────┐
   │ arm_gazebo  │      │ arm_control │     │arm_moveit   │
   │             │      │             │     │   _config   │
   └──────┬──────┘      └──────┬──────┘     └──────┬──────┘
          │                    │                    │
          │                    │                    │
          └────────────────────┼────────────────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ arm_system_bringup  │
                    └──────────┬──────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   ┌─────────────┐      ┌─────────────┐     ┌─────────────┐
   │arm_perception│      │  arm_demos  │     │arm_gui_tools│
   └─────────────┘      └─────────────┘     └─────────────┘
```

**Legend:**
- [ACTIVE]: Fully functional package
- [PARTIAL]: Some components implemented
- [PLACEHOLDER]: Structure exists, minimal/no code
- [MINIMAL]: Basic structure only

### 2.3 Core Package Details

#### arm_description (Robot Model)

**Purpose:** URDF/Xacro robot model definitions

**Key Files:**
```
arm_description/
├── urdf/
│   ├── arm.urdf.xacro              # Main entry (use_sim argument)
│   ├── arm_gazebo.xacro            # Simulation variant
│   ├── arm_real.xacro              # Real hardware variant
│   ├── links/
│   │   └── arm_links.xacro         # 14 link definitions
│   ├── joints/
│   │   └── arm_joints.xacro        # 8 actuated joints
│   └── macros/
│       ├── ros2_control.xacro      # Control interfaces
│       └── depth_camera.xacro      # RGBD camera
├── meshes/
│   ├── visual/                     # 40+ visual STL files
│   └── collision/                  # 40+ collision STL files
└── config/
    └── ros2_control.yaml           # Controller configuration
```

**Robot Specifications:**
- **6 DOF arm joints** (shoulder pitch/roll/yaw, elbow, wrist pitch/roll)
- **2 DOF gripper** (2 fingers, 1 mimic joint)
- **1 RGBD camera** (mounted on base_link)
- **14 links** (base, shoulder, elbow, wrist, hand, fingers)
- **Switching mechanism:** `use_sim:=true/false` → loads arm_gazebo.xacro or arm_real.xacro

**Dependencies:** None (base package)

---

#### arm_control (Simple Control)

**Purpose:** Direct trajectory control without planning

**Key Files:**
```
arm_control/
├── scripts/
│   ├── motion_planner.py           # MotionPlanner class
│   ├── example.py                  # Example motions
│   └── test_simple.py              # Basic tests
├── config/
│   ├── controllers.yaml            # Controller definitions
│   └── actuator_specs.yaml         # Motor specifications
└── launch/
    ├── sim.launch.py               # Gazebo + controllers
    └── control.launch.py           # Controllers only
```

**Control API:**
```python
from motion_planner import MotionPlanner

planner = MotionPlanner()
planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3, 0.0])  # 6 joint positions
planner.home()                                      # Predefined pose
planner.ready()                                     # Another pose
```

**Controllers (defined in controllers.yaml):**
1. `joint_state_broadcaster` - Publishes `/joint_states` at 50 Hz
2. `arm_controller` - JointTrajectoryController for 6 arm joints
3. `hand_controller` - GripperActionController for gripper

**Dependencies:** ros2_control, controller_manager, joint_trajectory_controller

---

#### arm_moveit_config (MoveIt2 Planning)

**Purpose:** Motion planning with collision avoidance

**Key Files:**
```
arm_moveit_config/
├── config/
│   ├── arm_description.srdf        # Semantic robot description
│   ├── kinematics.yaml             # KDL IK solver config
│   ├── joint_limits.yaml           # Planning velocity/accel limits
│   ├── moveit_controllers.yaml     # Controller interface
│   ├── sensors_3d.yaml             # OctoMap point cloud config
│   ├── moveit.rviz                 # RViz configuration
│   └── pilz_cartesian_limits.yaml  # Cartesian planning limits
├── launch/
│   ├── demo.launch.py              # MoveIt + RViz
│   ├── move_group.launch.py        # Planning server only
│   └── moveit_rviz.launch.py       # RViz only
└── nodes/
    ├── planner_node.py             # Custom planner
    └── gazebo_shape_sync.py        # Sync Gazebo objects to planning scene
```

**MoveIt Configuration:**
- **Planning group:** "arm" (6 joints) + "hand" (gripper)
- **Predefined poses:** home, ready, open, close
- **IK solver:** KDL (50ms timeout, position-only)
- **Planning pipelines:** OMPL, Pilz, CHOMP, STOMP
- **Collision pairs:** 298 disabled pairs for performance
- **3D sensors:** Point cloud from `/camera/depth/points` → OctoMap

**Usage Example:**
```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="planner")
arm = moveit.get_planning_component("arm")
arm.set_goal(pose_name="home")
plan = arm.plan()
if plan:
    arm.execute()
```

**Dependencies:** arm_description, moveit_ros, octomap_server

---

#### arm_gazebo (Simulation)

**Purpose:** Gazebo Harmonic physics simulation

**Key Files:**
```
arm_gazebo/
├── worlds/
│   ├── lab.sdf                     # Main lab world
│   ├── lab-ldr.sdf                 # LDR variant
│   ├── obstacle_course.sdf         # Testing world
│   └── whole_arm_workspace.sdf     # Full workspace
├── models/                         # 15+ Gazebo models (tables, chairs, etc.)
├── launch/
│   ├── arm_world.launch.py         # Gazebo world + robot spawn
│   ├── spawn_arm.launch.py         # Robot + controllers only
│   └── headless_sim.launch.py      # No GUI
└── config/
    ├── controllers.yaml            # Gazebo controller config
    └── bridge.yaml                 # ROS-Gazebo bridge topics
```

**Simulation Components:**
- **Physics engine:** Gazebo Harmonic (gz_sim)
- **ros2_control plugin:** GazeboSimROS2ControlPlugin
- **Camera bridge:** RGB, depth, point cloud topics
- **Clock bridge:** Synchronizes ROS time with Gazebo time

**Launch Sequence (spawn_arm.launch.py):**
```
t=0s:  robot_state_publisher launched
t=1s:  spawn_entity (robot in Gazebo)
t=2s:  camera_bridge (RGBD topics)
t=3s:  robot_state_publisher (delay for Gazebo ready)
t=4s:  joint_state_broadcaster spawned
t=5s:  arm_controller spawned
t=6s:  hand_controller spawned
```

**Dependencies:** arm_description, ros_gz_sim, ros_gz_bridge, gz_ros2_control

---

#### arm_perception (3D Perception)

**Purpose:** Real-time 3D perception and object tracking

**Key Files:**
```
arm_perception/
├── scripts/
│   ├── perception_node.py          # Point cloud processing
│   ├── object_recognition_node.py  # YOLO detection
│   ├── dynamic_object_tracker.py   # Kalman tracking
│   └── planning_scene_updater.py   # MoveIt scene sync
├── config/
│   ├── perception.yaml             # Processing parameters
│   └── object_recognition.yaml     # YOLO config
└── launch/
    └── perception.launch.py        # Full perception pipeline
```

**Perception Pipeline:**
```
Point Cloud Input (/camera/depth/points)
    │
    ▼
┌─────────────────────┐
│  Plane Removal      │  (Remove floor/walls, 1cm threshold)
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Clustering         │  (5cm tolerance, 200-10K points)
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Shape Detection    │  (Bounding boxes)
└─────────┬───────────┘
          │
          ├──────────────────────┐
          ▼                      ▼
┌─────────────────────┐  ┌─────────────────────┐
│  YOLO Recognition   │  │  Object Tracker     │
└─────────┬───────────┘  └─────────┬───────────┘
          │                        │
          └────────────┬───────────┘
                       ▼
          ┌─────────────────────────┐
          │  Planning Scene Updater │
          └─────────────────────────┘
                       │
                       ▼
              MoveIt Planning Scene
```

**Processing Parameters (perception.yaml):**
- Plane segmentation: 1cm threshold
- Clustering: 5cm tolerance, 200-10,000 points
- Processing rate: 1 Hz
- Z-axis filtering: 0-1.5m height

**Published Topics:**
- `/detected_objects` (visualization_msgs/MarkerArray)
- `/tracked_objects` (custom tracking messages)

**Dependencies:** pcl_ros, octomap_msgs, moveit_msgs, cv_bridge

---

#### arm_system_bringup (System Integration)

**Purpose:** Complete system launch orchestration

**Key Files:**
```
arm_system_bringup/
├── launch/
│   ├── full_system.launch.py                  # Headless sim + controllers (RECOMMENDED)
│   ├── moveit_gazebo.launch.py                # Gazebo + MoveIt + RViz (legacy)
│   ├── moveit_gazebo_with_octomap.launch.py   # + Perception (legacy)
│   ├── headless.launch.py                     # No GUI
│   ├── moveit_demo.launch.py                  # MoveIt demo
│   ├── moveit_octomap_only.launch.py          # OctoMap only
│   └── octomap_server.launch.py               # OctoMap server
└── config/
    ├── octomap_server.yaml                    # OctoMap parameters
    └── gazebo_bridge.yaml                     # ROS-Gazebo topics
```

**Primary Launch File (full_system.launch.py):**
```
full_system.launch.py (RECOMMENDED)
├── headless_sim.launch.py (arm_gazebo)
│   ├── gz_sim (Gazebo Harmonic headless)
│   ├── robot spawn
│   └── camera bridge
├── control.launch.py (arm_control)
│   ├── joint_state_broadcaster
│   ├── arm_controller
│   └── hand_controller
└── planner.launch.py (arm_moveit_config) [optional, commented out]
```

**Legacy Launch Hierarchy (moveit_gazebo.launch.py):**
```
moveit_gazebo.launch.py
├── arm_world.launch.py (arm_gazebo)
│   ├── gz_sim (Gazebo Harmonic with GUI)
│   └── spawn_arm.launch.py
├── move_group.launch.py (arm_moveit_config)
│   │   └── RViz2
│   ├── octomap_server.launch.py
│   └── perception.launch.py (arm_perception)
```

**Initialization Timing (moveit_gazebo.launch.py):**
```
t=0s:   Gazebo world + robot spawn + robot_state_publisher
t=3s:   joint_state_broadcaster
t=4s:   arm_controller
t=6s:   move_group (MoveIt planning server)
t=8s:   RViz2
t=10s:  System ready
```

**Dependencies:** All active packages

---

#### arm_gui_tools (GUI Tools - RECOMMENDED LAUNCHER)

**Purpose:** PyQt5-based graphical tools for system management

**Key Files:**
```
arm_gui_tools/
├── src/arm_gui_tools/
│   ├── full_system_launcher.py      # Main system launcher (PRIMARY)
│   └── joint_monitor.py             # Joint state visualization
├── ui/
│   ├── full_system_launcher.ui      # Qt Designer UI file
│   └── images/
│       └── image.jpg                # Branding image
└── launch/
    └── (no launch files - run directly)
```

**Full System Launcher Features:**

The `full_system_launcher.py` is the **PRIMARY RECOMMENDED METHOD** for launching the system.

**Launch command:**
```bash
ros2 run arm_gui_tools full_system_launcher.py
```

**GUI Components:**

1. **World Selection Dropdown**
   - Automatically discovers all .sdf files in arm_gazebo/worlds/
   - Displays: lab.sdf, obstacle_course.sdf, whole_arm_workspace.sdf, etc.
   - Selected world passed to full_system.launch.py via `simulation_world` argument

2. **Full System Control**
   - **Start button**: Launches `ros2 launch arm_system_bringup full_system.launch.py simulation_world:=<path>`
   - **Stop button**: Terminates the full system process
   - **Status label**: Shows PID and running state

3. **Individual Component Controls** (independent start/stop buttons):
   - **Gazebo GUI**: `gz sim -g` - Opens Gazebo GUI viewer for headless simulation
   - **RViz**: `rviz2 -d <perception_config>` - 3D visualization with camera view
   - **MoveIt**: `ros2 launch arm_moveit_config demo.launch.py` - MoveIt planning interface
   - **OctoMap**: `ros2 launch arm_system_bringup moveit_octomap_only.launch.py` - 3D occupancy mapping
   - **Object Detection**: `ros2 run arm_perception object_recognition_node.py` - YOLO detection
   - **Image Viewer**: `ros2 run image_tools showimage` - Camera feed viewer

4. **Process Management**
   - Each component runs as independent background process
   - Real-time status monitoring (PID, running/stopped/exited)
   - Automatic cleanup of finished processes
   - SIGINT/SIGTERM handling for graceful shutdown

**Workflow:**
```
1. Launch GUI
2. Select simulation world from dropdown
3. Click "Start" for full system (headless Gazebo + controllers)
4. Optionally click individual component buttons:
   - "Start" Gazebo GUI to view simulation
   - "Start" RViz for visualization
   - "Start" MoveIt for planning interface
5. Click "Stop" buttons to terminate components when done
```

**Architecture Benefits:**
- **Headless-first**: Gazebo runs headless by default, GUI on demand (saves resources)
- **Component independence**: Start/stop individual components without affecting others
- **Visual feedback**: Clear status for each component
- **Simplified workflow**: No need to manage multiple terminals
- **Process isolation**: Each component in separate process group

**Dependencies:** PyQt5, ros2, ament_index_python

**See Also:** [Section 5 - Launch System Architecture](#5-launch-system-architecture) for detailed launch strategies

---

## 3. Control Architecture

### 3.1 Dual Control Paradigm

The system provides **two independent control paths** that share the same underlying controller:

```
┌─────────────────────────────────────────────────────────────────┐
│                    USER CODE                                    │
│                                                                 │
│  ┌───────────────────┐              ┌─────────────────────┐   │
│  │  Simple Control   │              │  MoveIt Control     │   │
│  │  (motion_planner) │              │  (moveit_py)        │   │
│  │                   │              │                     │   │
│  │  - Direct joint   │              │  - IK solving       │   │
│  │  - Fast           │              │  - Collision avoid  │   │
│  │  - Known paths    │              │  - Cartesian        │   │
│  └─────────┬─────────┘              └──────────┬──────────┘   │
└────────────┼──────────────────────────────────┼───────────────┘
             │                                  │
             │      ┌───────────────────────────┘
             │      │
             ▼      ▼
    ┌────────────────────────────────────┐
    │  /arm_controller/                  │
    │  follow_joint_trajectory           │
    │  (FollowJointTrajectory Action)    │
    └─────────────┬──────────────────────┘
                  │
                  ▼
    ┌────────────────────────────────────┐
    │  ros2_control Framework            │
    │  ┌──────────────────────────────┐  │
    │  │  Controller Manager          │  │
    │  │  ┌────────────────────────┐  │  │
    │  │  │  arm_controller        │  │  │
    │  │  │  (JointTrajectory)     │  │  │
    │  │  └────────────────────────┘  │  │
    │  └──────────────────────────────┘  │
    └─────────────┬──────────────────────┘
                  │
                  ▼
    ┌────────────────────────────────────┐
    │  Hardware Abstraction              │
    │  ┌────────────┐   ┌─────────────┐  │
    │  │  Gazebo    │OR │  Hardware   │  │
    │  │  Plugin    │   │  Interface  │  │
    │  └────────────┘   └─────────────┘  │
    └────────────────────────────────────┘
```

### 3.2 Controller Stack Details

**Controller Manager:**
- Spawns/stops/configures controllers
- Manages hardware interfaces
- Provides controller lifecycle management

**Active Controllers:**

1. **joint_state_broadcaster**
   - Type: `JointStateBroadcaster`
   - Publish rate: 50 Hz
   - Topic: `/joint_states` (sensor_msgs/JointState)
   - Purpose: Publish current joint positions/velocities

2. **arm_controller**
   - Type: `JointTrajectoryController`
   - Joints: 6 arm joints
   - Action: `/arm_controller/follow_joint_trajectory`
   - Interfaces: position, velocity
   - Purpose: Execute arm trajectories

3. **hand_controller**
   - Type: `GripperActionController`
   - Joints: 2 gripper fingers (1 mimic)
   - Action: `/hand_controller/gripper_cmd`
   - Purpose: Open/close gripper

**Hardware Interfaces (defined in ros2_control.xacro):**
```xml
<joint name="left_shoulder_pitch_rs04">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
<!-- Repeated for all 8 joints -->
```

### 3.3 Control Flow Comparison

| Feature | Simple Control | MoveIt Control |
|---------|---------------|----------------|
| **Planning** | None (direct trajectory) | OMPL/Pilz/CHOMP/STOMP |
| **Collision Avoidance** | No | Yes (OctoMap) |
| **IK Solving** | Manual | Automatic (KDL) |
| **Cartesian Planning** | No | Yes |
| **Speed** | Fast (µs) | Slower (ms-s) |
| **Use Case** | Known motions | Unknown environments |
| **Complexity** | Low | High |

---

## 4. Data Flow Architecture

### 4.1 Topic Data Flow

```
                    PERCEPTION LAYER
                         │
    ┌────────────────────┼────────────────────┐
    │                    │                    │
    ▼                    ▼                    ▼
/camera/           /camera/              /camera/
color/image_raw    depth/image_raw       depth/points
(RGB)              (Depth)               (PointCloud2)
    │                    │                    │
    │                    └─────────┬──────────┘
    │                              │
    │                              ▼
    │                    ┌─────────────────────┐
    │                    │  perception_node    │
    │                    │  (Point cloud proc) │
    │                    └──────────┬──────────┘
    │                               │
    │                               ▼
    │                    ┌─────────────────────┐
    │                    │ /detected_objects   │
    │                    │ (MarkerArray)       │
    │                    └──────────┬──────────┘
    │                               │
    │                               ▼
    │                    ┌─────────────────────────┐
    │                    │ planning_scene_updater  │
    │                    │ (Sync to MoveIt)        │
    │                    └──────────┬──────────────┘
    │                               │
    │                               ▼
    │                    ┌─────────────────────┐
    │                    │  /planning_scene    │
    │                    │  (PlanningScene)    │
    │                    └─────────────────────┘
    │
    └─────────────────────────────────┐
                                      │
                    CONTROL LAYER     │
                         │            │
    ┌────────────────────┼────────────┼────────────────────┐
    │                    │            │                    │
    ▼                    ▼            ▼                    ▼
/joint_states      /arm_controller/  /hand_controller/  /robot_description
(JointState)       follow_joint_     gripper_cmd        (String - URDF)
                   trajectory        (GripperCommand)
    │              (FollowJointTraj) │                    │
    │                    │            │                    │
    │                    └────────────┼────────────────────┘
    │                                 │
    │                                 │
    ▼                                 ▼
┌─────────────────┐       ┌─────────────────────┐
│ robot_state_    │       │  Controllers        │
│ publisher       │       │  (ros2_control)     │
│ (TF broadcast)  │       │                     │
└─────────────────┘       └─────────────────────┘
    │                                 │
    ▼                                 ▼
/tf, /tf_static           Gazebo / Hardware
(TF2 transforms)          (Joint commands)
```

### 4.2 Action Servers

```
User Code
    │
    ├─────────────────────────────────┐
    │                                 │
    ▼                                 ▼
/arm_controller/              /hand_controller/
follow_joint_trajectory       gripper_cmd
(FollowJointTrajectory)       (GripperCommand)
    │                                 │
    │                                 │
    ▼                                 ▼
arm_controller               hand_controller
(JointTrajectoryController)  (GripperActionController)
    │                                 │
    │                                 │
    └─────────────┬───────────────────┘
                  │
                  ▼
        Hardware Interface
        (Gazebo or Real)
```

### 4.3 Service Interfaces

**Controller Manager Services:**
- `/controller_manager/list_controllers`
- `/controller_manager/load_controller`
- `/controller_manager/switch_controller`
- `/controller_manager/configure_controller`

**MoveIt Services:**
- `/get_planning_scene` (moveit_msgs/GetPlanningScene)
- `/apply_planning_scene` (moveit_msgs/ApplyPlanningScene)
- `/compute_ik` (moveit_msgs/GetPositionIK)
- `/compute_fk` (moveit_msgs/GetPositionFK)

### 4.4 TF Transform Tree

```
                        map
                         │
                         ▼
                    base_link (fixed to world)
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
    camera_link   shoulder_actuator   (other links...)
        │
        ├─────────────┬──────────────┐
        │             │              │
        ▼             ▼              ▼
  camera_rgb    camera_depth   camera_optical
    _frame         _frame          _frame

Full arm chain:
base_link
 └─ left_shoulder_pitch_link
     └─ left_shoulder_roll_link
         └─ left_shoulder_yaw_link
             └─ left_elbow_link
                 └─ left_wrist_link
                     └─ left_hand_link
                         └─ left_palm_link
                             ├─ left_finger_right_link
                             └─ left_finger_left_link
```

---

## 5. Launch System Architecture

### 5.1 Launch Strategies

The system supports **four launch strategies**:

#### Strategy 1: GUI Launcher (PRIMARY - RECOMMENDED)

```bash
ros2 run arm_gui_tools full_system_launcher.py
```

**What it does:**
1. Opens PyQt5 GUI with component controls
2. Select simulation world from dropdown
3. Click "Start" to launch full_system.launch.py (headless Gazebo + controllers)
4. Optionally start individual components via GUI buttons (Gazebo GUI, RViz, MoveIt, etc.)

**Advantages:**
- **User-friendly**: Visual interface, no terminal management
- **Flexible**: Start/stop individual components independently
- **Resource-efficient**: Headless simulation by default, GUIs on demand
- **Process management**: Automatic cleanup and status monitoring
- **World selection**: Easy dropdown menu for choosing simulation world

**Best for:** All users, especially beginners and demonstrations

**See Also:** [Section 2.3 - arm_gui_tools](#arm_gui_tools-gui-tools---recommended-launcher) for detailed GUI features

---

#### Strategy 2: Command-Line Full System (Alternative)

```bash
# With default world
ros2 launch arm_system_bringup full_system.launch.py

# With specific world
ros2 launch arm_system_bringup full_system.launch.py \
  simulation_world:=/path/to/world.sdf
```

**What it includes:**
- Headless Gazebo simulation
- Robot controllers (joint_state_broadcaster, arm_controller, hand_controller)
- Optional planner (commented out by default)

**Advantages:**
- Single command
- Good for scripting and automation
- Headless by default (lightweight)

**Best for:** Scripting, automation, headless servers

**File: [arm_system_bringup/launch/full_system.launch.py](src/bringup/arm_system_bringup/launch/full_system.launch.py)**

---

#### Strategy 3: Legacy All-in-One (For Reference)

```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

**Advantages:**
- Single command with MoveIt + RViz
- Automated timing/delays
- Complete system guaranteed

**Disadvantages:**
- Heavy (launches all GUIs)
- Less flexible than GUI launcher

**File: [arm_system_bringup/launch/moveit_gazebo.launch.py](src/bringup/arm_system_bringup/launch/moveit_gazebo.launch.py)**

---

#### Strategy 4: Individual Components (Advanced Debugging)

```bash
# Terminal 1: Gazebo world
ros2 launch arm_gazebo arm_world.launch.py

# Terminal 2: Spawn robot (wait 5s)
ros2 launch arm_gazebo spawn_arm.launch.py

# Terminal 3: MoveIt (wait 15s)
ros2 launch arm_moveit_config move_group.launch.py

# Terminal 4: RViz
ros2 launch arm_moveit_config moveit_rviz.launch.py
```

**Advantages:**
- Maximum control
- Best for debugging specific components

### 5.2 Launch File Composition

```
moveit_gazebo.launch.py
│
├─ IncludeLaunchDescription: arm_gazebo/arm_world.launch.py
│  │
│  ├─ ExecuteProcess: gz sim (Gazebo Harmonic)
│  ├─ Node: ros_gz_bridge (clock, /tf)
│  │
│  └─ IncludeLaunchDescription: spawn_arm.launch.py
│     │
│     ├─ Node: robot_state_publisher (t=0s)
│     ├─ Node: spawn_entity (t=1s, robot in Gazebo)
│     ├─ Node: ros_gz_bridge camera (t=2s, RGBD topics)
│     ├─ Node: robot_state_publisher again (t=3s, for safety)
│     ├─ LoadController: joint_state_broadcaster (t=4s)
│     ├─ LoadController: arm_controller (t=5s)
│     └─ LoadController: hand_controller (t=6s)
│
├─ TimerAction (6s delay): IncludeLaunchDescription: move_group.launch.py
│  │
│  ├─ Node: move_group (MoveIt planning server)
│  └─ Node: robot_state_publisher (for planning)
│
└─ TimerAction (8s delay): IncludeLaunchDescription: moveit_rviz.launch.py
   │
   └─ Node: rviz2 (with moveit.rviz config)
```

### 5.3 Critical Timing Dependencies

**Why delays are necessary:**

```
t=0s:   Gazebo starts (needs 3-5s to initialize)
        ↓
t=3s:   Controller manager available in Gazebo
        ↓ (can now spawn controllers)
t=4s:   joint_state_broadcaster active
        ↓ (/joint_states published)
t=5s:   arm_controller active
        ↓ (can accept trajectories)
t=6s:   move_group can start (needs /joint_states + controllers)
        ↓
t=8s:   RViz can start (needs move_group)
```

**Common failure modes without delays:**
- Controllers fail to spawn → controller_manager not ready
- move_group fails to start → no /joint_states
- RViz shows no robot → robot_state_publisher not ready
- Planning fails → controllers not active

---

## 6. Robot Model Architecture

### 6.1 URDF/Xacro Organization

```
arm.urdf.xacro (main entry)
│
├─ <xacro:arg name="use_sim" default="true"/>
│
├─ <xacro:include> arm_base.xacro
│  └─ Base robot structure
│
├─ <xacro:if value="${use_sim}">
│  └─ <xacro:include> arm_gazebo.xacro
│     ├─ GazeboSimROS2ControlPlugin
│     ├─ Gazebo-specific properties
│     └─ Camera sensor plugin
│
└─ <xacro:unless value="${use_sim}">
   └─ <xacro:include> arm_real.xacro
      └─ Real hardware ros2_control interface
```

**Switching mechanism:**
```bash
# Simulation mode
xacro arm.urdf.xacro use_sim:=true > arm_sim.urdf

# Real hardware mode
xacro arm.urdf.xacro use_sim:=false > arm_real.urdf
```

### 6.2 Link Structure (14 total)

```
base_link (fixed to world)
    │
    ├─ shoulder_actuator_link
    ├─ shoulder_flange_link
    ├─ shoulder_pitch_link
    ├─ shoulder_roll_link
    ├─ shoulder_yaw_link
    │
    ├─ elbow_actuator_link
    ├─ elbow_flange_link
    ├─ elbow_link
    │
    ├─ wrist_actuator_link
    ├─ wrist_flange_link
    │
    ├─ hand_link
    ├─ palm_link
    ├─ finger_right_link
    └─ finger_left_link
```

**Link properties:**
- Visual mesh: STL files in `meshes/visual/`
- Collision mesh: STL files in `meshes/collision/`
- Inertial properties: Mass, inertia tensor, center of mass

### 6.3 Joint Configuration (8 actuated)

**Arm Joints (6 DOF):**

| Joint Name | Type | Limits (rad) | Max Torque | Max Velocity |
|------------|------|--------------|------------|--------------|
| `left_shoulder_pitch_rs04` | revolute | [-3.14, 3.14] | 120 Nm | 3.0 rad/s |
| `left_shoulder_roll_rs04` | revolute | [-3.5, 0.6] | 120 Nm | 3.0 rad/s |
| `left_shoulder_yaw_rs03` | revolute | [-3.14, 3.14] | 60 Nm | 1.6 rad/s |
| `left_elbow_rs03` | revolute | [-1.3, 1.9] | 60 Nm | 1.6 rad/s |
| `left_wrist_rs02` | revolute | [-3.14, 3.14] | 17 Nm | 0.8 rad/s |
| `left_hand_rs02` | revolute | [-3.14, 3.14] | 17 Nm | 0.8 rad/s |

**Gripper Joints (2 DOF):**

| Joint Name | Type | Limits (rad) | Notes |
|------------|------|--------------|-------|
| `left_palm_right_finger` | revolute | [0, 0.8] | Main control |
| `left_palm_left_finger` | revolute | [0, 0.8] | Mimic (auto) |

### 6.4 Sensor Configuration

**RGBD Camera:**
- **Mount location:** base_link (1.75m height, 15° downward tilt)
- **Type:** Gazebo depth camera
- **Resolution:** 640x480
- **FOV:** 60° horizontal, 45° vertical
- **Range:** 0.3m - 10m
- **Frame rate:** 30 Hz

**Topics published:**
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/depth/points` (sensor_msgs/PointCloud2)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo)

---

## 7. Perception Architecture

### 7.1 Perception Pipeline Detail

```
┌─────────────────────────────────────────────────────────────────┐
│                    INPUT LAYER                                  │
│  /camera/depth/points (PointCloud2, 640x480, 30 Hz)            │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│              PREPROCESSING (perception_node.py)                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ 1. PassThrough Filter (Z-axis: 0-1.5m)                    │ │
│  │    - Remove points outside workspace                      │ │
│  └───────────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ 2. Voxel Grid Downsampling (1cm leaf size)                │ │
│  │    - Reduce point density for performance                 │ │
│  └───────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                 SEGMENTATION                                    │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ 3. Plane Segmentation (RANSAC, 1cm threshold)             │ │
│  │    - Remove floor, walls, tables                          │ │
│  │    - Max iterations: 100                                  │ │
│  └───────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                  CLUSTERING                                     │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ 4. Euclidean Clustering                                   │ │
│  │    - Tolerance: 5cm                                       │ │
│  │    - Min cluster size: 200 points                         │ │
│  │    - Max cluster size: 10,000 points                      │ │
│  └───────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│               SHAPE DETECTION                                   │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ 5. Bounding Box Computation                               │ │
│  │    - Axis-aligned bounding boxes (AABB)                   │ │
│  │    - Extract center, size, orientation                    │ │
│  └───────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ├──────────────────┐
                           │                  │
                           ▼                  ▼
    ┌───────────────────────────┐  ┌──────────────────────────┐
    │  RECOGNITION BRANCH       │  │  TRACKING BRANCH         │
    │                           │  │                          │
    │  object_recognition_      │  │  dynamic_object_         │
    │  node.py                  │  │  tracker.py              │
    │                           │  │                          │
    │  6. YOLO Detection        │  │  6. Kalman Filtering     │
    │     - YOLOv8 model        │  │     - Position predict   │
    │     - 80 COCO classes     │  │     - Velocity estimate  │
    │     - Confidence: 0.5     │  │     - Association        │
    └─────────────┬─────────────┘  └─────────────┬────────────┘
                  │                              │
                  └──────────────┬───────────────┘
                                 │
                                 ▼
                  ┌──────────────────────────────┐
                  │  /detected_objects           │
                  │  (MarkerArray)               │
                  └───────────────┬──────────────┘
                                  │
                                  ▼
                  ┌──────────────────────────────┐
                  │  planning_scene_updater.py   │
                  │                              │
                  │  7. MoveIt Scene Sync        │
                  │     - Add collision objects  │
                  │     - Update positions       │
                  │     - Remove old objects     │
                  └───────────────┬──────────────┘
                                  │
                                  ▼
                  ┌──────────────────────────────┐
                  │  /planning_scene             │
                  │  (PlanningScene)             │
                  │                              │
                  │  Used by MoveIt for          │
                  │  collision avoidance         │
                  └──────────────────────────────┘
```

### 7.2 OctoMap Integration

**OctoMap Server Configuration ([octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml)):**
```yaml
resolution: 0.05           # 5cm voxel size
frame_id: base_link        # Reference frame
max_range: 5.0             # Maximum sensor range
sensor_model:
  hit: 0.7                 # Probability for occupied
  miss: 0.4                # Probability for free
  min: 0.12                # Clamping min
  max: 0.97                # Clamping max
```

**OctoMap in MoveIt ([sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml)):**
```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/points
    max_range: 5.0
    padding_offset: 0.1
    padding_scale: 1.0
    filtered_cloud_topic: filtered_cloud
```

**Data flow:**
```
/camera/depth/points
    │
    ├─> octomap_server → /octomap_binary (OctomapBinary)
    │                  → /octomap_full (Octomap)
    │
    └─> move_group (OctomapUpdater) → Planning scene
                                     → Collision checking
```

---

## 8. Build System Architecture

### 8.1 Colcon Workspace Structure

```
ldr-humanoid-arm-system/
├── src/                        # Source packages
├── build/                      # Build artifacts (generated)
├── install/                    # Installed packages (generated)
├── log/                        # Build logs (generated)
└── colcon.meta (optional)      # Build configuration
```

### 8.2 Package Build Types

**Type 1: ament_cmake (C++ + Python)**
- arm_description
- arm_gazebo
- arm_control
- arm_moveit_config
- arm_system_bringup
- arm_teleop

**Type 2: ament_python (Pure Python)**
- manipulation_library
- arm_gui_tools

### 8.3 CMakeLists.txt Patterns

**Pattern 1: URDF Package**
```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**Pattern 2: Launch-Only Package**
```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_system_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**Pattern 3: Python Scripts Package**
```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motion_planner.py
  scripts/example.py
  scripts/test_simple.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

**Pattern 4: C++ Node Package**
```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(joint_teleop_node src/joint_teleop_node.cpp)
ament_target_dependencies(joint_teleop_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  control_msgs
)

install(TARGETS joint_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 8.4 package.xml Dependency Types

**1. Build Dependencies (`<depend>`):**
```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>moveit_ros_planning_interface</depend>
```

**2. Execution Dependencies (`<exec_depend>`):**
```xml
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
```

**3. Build Tool Dependencies (`<buildtool_depend>`):**
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
```

**4. Test Dependencies (`<test_depend>`):**
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

**5. Plugin Exports (`<export>`):**
```xml
<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros plugin_path="${prefix}/lib"/>
</export>
```

### 8.5 Build Commands

**Full build:**
```bash
colcon build
```

**Incremental build (specific package):**
```bash
colcon build --packages-select arm_control
```

**Parallel build:**
```bash
colcon build --parallel-workers 4
```

**Build with verbose output:**
```bash
colcon build --event-handlers console_direct+
```

**Clean build:**
```bash
rm -rf build/ install/ log/
colcon build
```

**Symlink install (for Python development):**
```bash
colcon build --symlink-install
```

---

## 9. Deployment Architectures

### 9.1 Simulation-Only Deployment

**Use case:** Development, testing, visualization

```
┌─────────────────────────────────────────────┐
│         Single Computer (Ubuntu 24.04)      │
│                                             │
│  ┌────────────────────────────────────────┐│
│  │  ROS 2 Jazzy Workspace                 ││
│  │                                         ││
│  │  ┌──────────┐  ┌──────────┐           ││
│  │  │ Gazebo   │  │ MoveIt2  │           ││
│  │  │ Harmonic │  │          │           ││
│  │  └──────────┘  └──────────┘           ││
│  │                                         ││
│  │  ┌──────────┐  ┌──────────┐           ││
│  │  │  RViz2   │  │Perception│           ││
│  │  └──────────┘  └──────────┘           ││
│  └────────────────────────────────────────┘│
└─────────────────────────────────────────────┘
```

**Launch command:**
```bash
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

### 9.2 Real Hardware Deployment

**Use case:** Physical robot control

```
┌─────────────────────────────────────────────┐
│      Control Computer (Ubuntu 24.04)        │
│                                             │
│  ┌────────────────────────────────────────┐│
│  │  ROS 2 Jazzy Workspace                 ││
│  │                                         ││
│  │  ┌──────────┐  ┌──────────┐           ││
│  │  │ MoveIt2  │  │Perception│           ││
│  │  │          │  │          │           ││
│  │  └──────────┘  └──────────┘           ││
│  │                                         ││
│  │  ┌──────────┐  ┌──────────┐           ││
│  │  │arm_control│  │  RViz2   │           ││
│  │  └────┬─────┘  └──────────┘           ││
│  └───────┼────────────────────────────────┘│
└──────────┼──────────────────────────────────┘
           │ (Ethernet/USB/CAN)
           │
┌──────────▼──────────────────────────────────┐
│     Hardware Interface Computer             │
│                                             │
│  ┌────────────────────────────────────────┐│
│  │  arm_hardware plugin                   ││
│  │  (Real hardware ros2_control)          ││
│  └───────────────┬────────────────────────┘│
│                  │                          │
│  ┌───────────────▼────────────────────────┐│
│  │  Motor Drivers (CAN/Ethernet)          ││
│  └───────────────┬────────────────────────┘│
└──────────────────┼──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│          Physical Robot                     │
│  ┌────────────────────────────────────────┐│
│  │  6 Motors + 2 Gripper Motors           ││
│  │  RGBD Camera                            ││
│  │  Joint Encoders                         ││
│  └────────────────────────────────────────┘│
└─────────────────────────────────────────────┘
```

**Launch command:**
```bash
# On control computer
ros2 launch arm_system_bringup real_robot.launch.py use_sim:=false
```

**Note:** Real hardware launch files are placeholders. Implementation needed in:
- [arm_hardware](src/hardware_interface/arm_hardware/) package
- arm_real.xacro hardware interface
- Real robot launch files

### 9.3 Distributed Deployment

**Use case:** Multiple computers, remote operation

```
┌──────────────────────────────────────────────────────────────────┐
│                  NETWORK (ROS 2 DDS)                             │
└──────────────────────────────────────────────────────────────────┘
     │                    │                    │
     │                    │                    │
┌────▼───────┐    ┌──────▼──────┐    ┌───────▼──────┐
│ Computer 1 │    │ Computer 2  │    │ Computer 3   │
│            │    │             │    │              │
│ Gazebo/    │    │  MoveIt2    │    │   RViz2      │
│ Hardware   │    │  Planning   │    │   GUI        │
│ Controllers│    │  Perception │    │   Monitor    │
└────────────┘    └─────────────┘    └──────────────┘
```

**Configuration:**
- Set `ROS_DOMAIN_ID` on all computers
- Configure DDS discovery (multicast or unicast)
- Use `ROS_LOCALHOST_ONLY=0` for network communication

---

## 10. Future Extensions

### 10.1 Planned Package Implementations

**Short-term (active development needed):**

1. **arm_hardware** - Real hardware interface
   - Implement ros2_control `HardwareInterface`
   - Motor driver communication (CAN/Ethernet)
   - Encoder reading
   - Safety limits

2. **gripper_control** - Gripper-specific control
   - Force control
   - Object grasp detection
   - Adaptive grasp

3. **manipulation_library** - Manipulation primitives
   - Pick and place
   - Push/pull
   - Insert/extract

4. **diagnostic_tools** - System monitoring
   - Joint health monitoring
   - Performance metrics
   - Error logging

**Long-term (future expansion):**

1. **hand_description** - Dexterous hand
   - Multi-finger hand model
   - Tactile sensors

2. **dual_arm_description** - Dual-arm configuration
   - Two arms coordination
   - Bimanual manipulation

3. **bimanual_demos** - Dual-arm demonstrations
   - Coordinated pick-place
   - Assembly tasks

### 10.2 Architecture Extension Points

**1. Adding new controllers:**
```yaml
# In controllers.yaml
new_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  ros__parameters:
    joints:
      - new_joint_1
      - new_joint_2
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

**2. Adding new planning pipelines:**
```yaml
# In moveit_config/config/ompl_planning.yaml
new_planner:
  type: geometric::RRT
  range: 0.1
  goal_bias: 0.05
```

**3. Adding new perception nodes:**
```python
# In arm_perception/scripts/
class NewPerceptionNode(Node):
    def __init__(self):
        super().__init__('new_perception_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.callback,
            10
        )
```

**4. Adding new sensors:**
```xml
<!-- In depth_camera.xacro or create new sensor macro -->
<gazebo reference="sensor_link">
  <sensor name="new_sensor" type="new_sensor_type">
    <update_rate>30</update_rate>
    <!-- Sensor configuration -->
  </sensor>
</gazebo>
```

### 10.3 Scalability Considerations

**Computational requirements:**
- **Minimum:** 4 CPU cores, 8GB RAM (simulation only)
- **Recommended:** 8 CPU cores, 16GB RAM, GPU (full system with perception)
- **GPU:** Beneficial for YOLO object recognition

**Network bandwidth (distributed deployment):**
- Point cloud (640x480 @ 30Hz): ~30 MB/s
- Joint states (50 Hz): ~10 KB/s
- Camera images (30 Hz): ~10 MB/s

**Optimization strategies:**
- Reduce point cloud resolution
- Lower perception processing rate
- Disable unused topics
- Use compressed image transport

---

## Appendix A: File Reference Index

### Key Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| controllers.yaml | Controller definitions | [arm_control/config/](src/control/arm_control/config/controllers.yaml) |
| kinematics.yaml | IK solver config | [arm_moveit_config/config/](src/planning/arm_moveit_config/config/kinematics.yaml) |
| joint_limits.yaml | Planning limits | [arm_moveit_config/config/](src/planning/arm_moveit_config/config/joint_limits.yaml) |
| arm_description.srdf | Semantic robot | [arm_moveit_config/config/](src/planning/arm_moveit_config/config/arm_description.srdf) |
| perception.yaml | Perception params | [arm_perception/config/](src/perception/arm_perception/config/perception.yaml) |
| octomap_server.yaml | OctoMap config | [arm_system_bringup/config/](src/bringup/arm_system_bringup/config/octomap_server.yaml) |

### Key Launch Files

| File | Purpose | Location |
|------|---------|----------|
| sim.launch.py | Gazebo + controllers | [arm_control/launch/](src/control/arm_control/launch/sim.launch.py) |
| demo.launch.py | MoveIt + RViz | [arm_moveit_config/launch/](src/planning/arm_moveit_config/launch/demo.launch.py) |
| moveit_gazebo.launch.py | Complete system | [arm_system_bringup/launch/](src/bringup/arm_system_bringup/launch/moveit_gazebo.launch.py) |
| arm_world.launch.py | Gazebo world | [arm_gazebo/launch/](src/simulation/arm_gazebo/launch/arm_world.launch.py) |
| perception.launch.py | Perception pipeline | [arm_perception/launch/](src/perception/arm_perception/launch/perception.launch.py) |

### Key Python Scripts

| File | Purpose | Location |
|------|---------|----------|
| motion_planner.py | Simple control API | [arm_control/scripts/](src/control/arm_control/scripts/motion_planner.py) |
| perception_node.py | Point cloud processing | [arm_perception/scripts/](src/perception/arm_perception/scripts/perception_node.py) |
| planner_node.py | Custom MoveIt planner | [arm_moveit_config/nodes/](src/planning/arm_moveit_config/nodes/planner_node.py) |

### Key URDF/Xacro Files

| File | Purpose | Location |
|------|---------|----------|
| arm.urdf.xacro | Main robot model | [arm_description/urdf/](src/robot_description/arm_description/urdf/arm.urdf.xacro) |
| arm_gazebo.xacro | Simulation variant | [arm_description/urdf/](src/robot_description/arm_description/urdf/arm_gazebo.xacro) |
| ros2_control.xacro | Control interfaces | [arm_description/urdf/macros/](src/robot_description/arm_description/urdf/macros/ros2_control.xacro) |
| depth_camera.xacro | Camera sensor | [arm_description/urdf/macros/](src/robot_description/arm_description/urdf/macros/depth_camera.xacro) |

---

## Appendix B: Quick Reference Commands

### Build & Source
```bash
colcon build
source install/setup.bash
```

### Launch System
```bash
# Modular approach (recommended)
ros2 launch arm_control sim.launch.py
ros2 launch arm_moveit_config demo.launch.py

# All-in-one
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# With perception
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

### Run Examples
```bash
ros2 run arm_control example.py
ros2 run arm_control test_simple.py
```

### Check System Status
```bash
ros2 control list_controllers
ros2 topic list
ros2 topic echo /joint_states
ros2 service list
```

### Debug
```bash
# Check URDF
xacro src/robot_description/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf

# Monitor topics
ros2 topic hz /joint_states
ros2 topic bw /camera/depth/points

# Controller info
ros2 control list_hardware_interfaces
ros2 control list_controller_types
```

---

**End of Architecture Documentation**
