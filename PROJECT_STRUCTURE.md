# LDR Humanoid Arm System - Project Structure

Complete folder structure with all files (excluding build artifacts).

## Overview

```
src/
├── applications/          # High-level application demos
├── bringup/              # System launch configurations
├── control/              # ROS2 control packages
├── hardware_interface/   # Hardware interface implementations
├── planning/             # MoveIt2 motion planning configs
├── robot_description/    # URDF/Xacro robot models
├── simulation/           # Gazebo simulation setup
└── tools/                # GUI and diagnostic tools
```

---

## Detailed Structure

### 📦 Applications (`src/applications/`)

```
applications/
├── arm_demos/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
├── bimanual_demos/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
└── manipulation_library/
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    ├── manipulation_library/
    │   └── __init__.py
    └── src/manipulation_library/
        └── __init__.py
```

---

### 🚀 Bringup (`src/bringup/`)

```
bringup/
└── arm_system_bringup/
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md
    ├── config/
    │   └── .gitkeep
    ├── launch/
    │   ├── .gitkeep
    │   ├── moveit_demo.launch.py
    │   └── moveit_gazebo.launch.py
    └── scripts/
        └── .gitkeep
```

---

### 🎮 Control (`src/control/`) ⭐ REFACTORED

```
control/
├── arm_control/                        # ⭐ MAIN CONTROL PACKAGE (REFACTORED)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md                       # Usage guide
│   ├── QUICK_START.md                  # Quick reference
│   ├── MOVEIT_INTEGRATION.md           # MoveIt2 integration
│   ├── REFACTOR_SUMMARY.md             # What changed
│   ├── config/
│   │   ├── .gitkeep
│   │   └── controllers.yaml            # Simple controller config
│   ├── launch/
│   │   ├── .gitkeep
│   │   ├── control.launch.py           # Controllers only
│   │   └── sim.launch.py               # Complete simulation
│   └── scripts/
│       ├── .gitkeep
│       ├── motion_planner.py           # Simple motion planner API
│       ├── example.py                  # Usage examples
│       └── test_simple.py              # Quick tests
│
├── dual_arm_control/                   # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
├── gripper_control/                    # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
└── hand_control/                       # (Placeholder)
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── .gitkeep
    ├── launch/
    │   └── .gitkeep
    └── scripts/
        └── .gitkeep
```

---

### 🔌 Hardware Interface (`src/hardware_interface/`)

```
hardware_interface/
├── arm_hardware/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── include/arm_hardware/
│   │   └── .gitkeep
│   ├── src/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
├── gripper_hardware/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── include/gripper_hardware/
│   │   └── .gitkeep
│   ├── src/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   └── scripts/
│       └── .gitkeep
│
└── hand_hardware/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── .gitkeep
    ├── include/hand_hardware/
    │   └── .gitkeep
    ├── src/
    │   └── .gitkeep
    ├── launch/
    │   └── .gitkeep
    └── scripts/
        └── .gitkeep
```

---

### 🧭 Planning (`src/planning/`)

```
planning/
├── arm_moveit_config/                  # ⭐ MAIN MOVEIT2 CONFIG
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── .setup_assistant
│   ├── config/
│   │   ├── arm_description.ros2_control.xacro
│   │   ├── arm_description.srdf        # Semantic description
│   │   ├── arm_description.urdf.xacro
│   │   ├── initial_positions.yaml
│   │   ├── joint_limits.yaml           # Planning limits
│   │   ├── kinematics.yaml             # IK solver config
│   │   ├── moveit_controllers.yaml     # Controller interface
│   │   ├── moveit.rviz                 # RViz config
│   │   ├── pilz_cartesian_limits.yaml
│   │   └── ros2_controllers.yaml       # ROS2 control config
│   └── launch/
│       ├── demo.launch.py              # Interactive demo
│       ├── move_group.launch.py        # Planning node
│       ├── moveit_rviz.launch.py       # Visualization
│       ├── rsp.launch.py               # Robot state publisher
│       ├── setup_assistant.launch.py   # Setup wizard
│       ├── spawn_controllers.launch.py
│       ├── static_virtual_joint_tfs.launch.py
│       └── warehouse_db.launch.py
│
├── arm_gripper_moveit_config/          # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   └── launch/
│       └── .gitkeep
│
├── arm_hand_moveit_config/             # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   └── launch/
│       └── .gitkeep
│
└── dual_arm_moveit_config/             # (Placeholder)
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── .gitkeep
    └── launch/
        └── .gitkeep
```

---

### 🤖 Robot Description (`src/robot_description/`)

```
robot_description/
├── arm_description/                    # ⭐ MAIN ROBOT MODEL
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── config/
│   │   ├── .gitkeep
│   │   └── view_robot.rviz
│   ├── launch/
│   │   ├── .gitkeep
│   │   ├── display_arm.launch.py
│   │   └── view_arm.launch.py
│   ├── meshes/
│   │   ├── collision/                  # 14 STL files
│   │   │   ├── cot1.STL
│   │   │   ├── cot2.STL
│   │   │   ├── cot3.STL
│   │   │   ├── flansa .STL
│   │   │   ├── incheietura.STL
│   │   │   ├── pmotor1.STL
│   │   │   ├── pmotor2.STL
│   │   │   ├── pmotor3.STL
│   │   │   ├── pmotor4.STL
│   │   │   ├── pmotor5.STL
│   │   │   ├── pmotor6.STL
│   │   │   ├── suport.STL
│   │   │   ├── umar1.STL
│   │   │   └── umar2.STL
│   │   └── visual/                     # 14 STL files (same names)
│   │       └── [same as collision]
│   └── urdf/
│       ├── arm.urdf.xacro              # Main entry point
│       ├── arm_base.xacro              # Base structure
│       ├── arm_gazebo.xacro            # Gazebo-specific
│       ├── arm_real.xacro              # Real hardware
│       ├── humanoid_arm_5dof.urdf      # Static URDF
│       ├── joints/
│       │   └── arm_joints.xacro        # Joint definitions
│       ├── links/
│       │   └── arm_links.xacro         # Link definitions
│       └── macros/
│           ├── gazebo_extensions.xacro # Gazebo plugins
│           ├── materials.xacro         # Visual materials
│           └── ros2_control.xacro      # ROS2 control interface
│
├── dual_arm_description/               # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   └── urdf/
│       └── .gitkeep
│
├── gripper_description/                # (Placeholder)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── .gitkeep
│   ├── launch/
│   │   └── .gitkeep
│   ├── meshes/
│   │   ├── collision/
│   │   │   └── .gitkeep
│   │   └── visual/
│   │       └── .gitkeep
│   └── urdf/
│       └── .gitkeep
│
└── hand_description/                   # (Placeholder)
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── .gitkeep
    ├── launch/
    │   └── .gitkeep
    ├── meshes/
    │   ├── collision/
    │   │   └── .gitkeep
    │   └── visual/
    │       └── .gitkeep
    └── urdf/
        └── .gitkeep
```

---

### 🌍 Simulation (`src/simulation/`)

```
simulation/
├── arm_gazebo/                         # ⭐ GAZEBO SIMULATION
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── .gitkeep
│   │   ├── controllers.yaml            # Gazebo controllers
│   │   └── gazebo_ros2_control.yaml    # (Deprecated)
│   ├── launch/
│   │   ├── .gitkeep
│   │   ├── arm_world.launch.py         # Launch Gazebo world
│   │   └── spawn_arm.launch.py         # Spawn arm robot
│   └── worlds/
│       ├── .gitkeep
│       └── lab.sdf                     # Lab environment
│
└── Gz1.md                              # Gazebo documentation
```

---

### 🛠️ Tools (`src/tools/`)

```
tools/
├── arm_gui_tools/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── setup.py
│   ├── arm_gui_tools/
│   │   └── __init__.py
│   ├── src/arm_gui_tools/
│   │   └── __init__.py
│   ├── launch/
│   │   └── .gitkeep
│   └── ui/
│       └── .gitkeep
│
└── diagnostic_tools/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── .gitkeep
    └── scripts/
        └── .gitkeep
```

---

## Key Packages Summary

| Package | Type | Status | Purpose |
|---------|------|--------|---------|
| `arm_control` | Control | ✅ **Active** | Simple motion planner + controllers |
| `arm_moveit_config` | Planning | ✅ **Active** | MoveIt2 configuration |
| `arm_description` | Description | ✅ **Active** | 5-DOF arm URDF model |
| `arm_gazebo` | Simulation | ✅ **Active** | Gazebo Harmonic simulation |
| `arm_system_bringup` | Bringup | ✅ **Active** | System launch files |
| `arm_hardware` | Hardware | 🔶 **Placeholder** | Real hardware interface |
| `dual_arm_control` | Control | 🔶 **Placeholder** | Dual-arm control |
| `gripper_*` | Various | 🔶 **Placeholder** | Gripper packages |
| `hand_*` | Various | 🔶 **Placeholder** | Hand packages |

---

## Important Files Reference

### Control & Simulation
- `src/control/arm_control/scripts/motion_planner.py` - Simple motion planner API
- `src/control/arm_control/launch/sim.launch.py` - Complete simulation launcher
- `src/control/arm_control/config/controllers.yaml` - Controller configuration
- `src/simulation/arm_gazebo/launch/arm_world.launch.py` - Gazebo world
- `src/simulation/arm_gazebo/worlds/lab.sdf` - Simulation environment

### Planning
- `src/planning/arm_moveit_config/launch/demo.launch.py` - MoveIt2 interactive demo
- `src/planning/arm_moveit_config/config/moveit_controllers.yaml` - MoveIt controller interface
- `src/planning/arm_moveit_config/config/kinematics.yaml` - IK solver settings

### Robot Model
- `src/robot_description/arm_description/urdf/arm.urdf.xacro` - Main robot model
- `src/robot_description/arm_description/urdf/arm_gazebo.xacro` - Gazebo config
- `src/robot_description/arm_description/urdf/macros/ros2_control.xacro` - Control interface

### Documentation
- `src/control/arm_control/README.md` - Control usage guide
- `src/control/arm_control/MOVEIT_INTEGRATION.md` - MoveIt2 integration
- `src/control/arm_control/QUICK_START.md` - Quick reference
- `src/control/arm_control/REFACTOR_SUMMARY.md` - Refactor details

---

## Quick Navigation

**Want to:**
- **Control the robot?** → See `src/control/arm_control/`
- **Use MoveIt2?** → See `src/planning/arm_moveit_config/`
- **Modify robot model?** → See `src/robot_description/arm_description/`
- **Configure simulation?** → See `src/simulation/arm_gazebo/`
- **Launch system?** → See `src/bringup/arm_system_bringup/`

---

**Total Packages:** 25
**Active Packages:** 5
**Placeholder Packages:** 20

*Generated: 2025-10-30*
