# LDR Humanoid Arm System - Documentation

**Complete technical documentation for the ROS 2 Jazzy robotics control system**

---

## 📚 Documentation Index

### Core Architecture Documents

| Document | Description | Best For |
|----------|-------------|----------|
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Complete system architecture specification | Understanding the full system design, package organization, and technical details |
| **[ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md)** | Visual architecture diagrams (Mermaid) | Visual learners, presentations, quick reference |

### Quick Start Guide

New to the project? Start here:

1. **Read [CLAUDE.md](../CLAUDE.md)** - Project overview and GUI launcher quick start
2. **Launch the system** - `ros2 run arm_gui_tools full_system_launcher.py` (recommended)
3. **Review [ARCHITECTURE.md](ARCHITECTURE.md) Section 1** - System overview
4. **View [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) Diagrams 1-3** - Visual system understanding

---

## 🏗️ Architecture Overview

### System Layers

The LDR Humanoid Arm System is organized into **5 architectural layers**:

```
┌─────────────────────────────────────────┐
│  User Interface (RViz, GUI, Teleop)     │
├─────────────────────────────────────────┤
│  Application (MoveIt2, Perception)      │
├─────────────────────────────────────────┤
│  Control (ros2_control, Controllers)    │
├─────────────────────────────────────────┤
│  Hardware Abstraction (Gazebo/Real)     │
├─────────────────────────────────────────┤
│  Physical (Simulation or Robot)         │
└─────────────────────────────────────────┘
```

### Package Organization

**25 ROS 2 packages** across **9 categories**:

- **Robot Description** (4) - URDF models
- **Control** (5) - Motion control interfaces
- **Planning** (5) - MoveIt2 configuration
- **Simulation** (1) - Gazebo Harmonic
- **Perception** (2) - 3D vision processing
- **Hardware** (3) - Real hardware interfaces (placeholder)
- **Bringup** (1) - System integration
- **Applications** (3) - Demo applications
- **Tools** (2) - Development utilities

**Active packages:** 8 fully functional
**Placeholder packages:** 17 for future expansion

---

## 🎯 Key Architecture Features

### 1. Dual Control Paradigm

Two independent control approaches sharing the same controller:

**Simple Control** (`motion_planner.py`)
- Direct joint trajectory control
- Fast execution (microseconds)
- Good for known motions

**MoveIt Control** (`moveit_py`)
- Inverse kinematics
- Collision avoidance
- Cartesian planning

### 2. Modular Launch System

Three launch strategies:

**Modular** (Recommended for development)
```bash
# Terminal 1
ros2 launch arm_control sim.launch.py

# Terminal 2 (wait 20s)
ros2 launch arm_moveit_config demo.launch.py
```

**All-in-One** (Recommended for demos)
```bash
ros2 launch arm_system_bringup moveit_gazebo.launch.py
```

**Individual Components** (Advanced debugging)
```bash
# Launch Gazebo, spawn, MoveIt, RViz separately
```

### 3. Perception Pipeline

Real-time 3D perception:
- Point cloud processing (PCL)
- Plane removal and clustering
- YOLO object recognition
- Dynamic object tracking (Kalman filtering)
- OctoMap integration for collision avoidance

### 4. Simulation-First Design

Single argument switches between simulation and real hardware:
```bash
# Simulation
ros2 launch ... use_sim:=true

# Real hardware (when implemented)
ros2 launch ... use_sim:=false
```

---

## 📖 Document Deep Dive

### ARCHITECTURE.md Sections

| Section | Content | When to Read |
|---------|---------|--------------|
| 1. System Overview | High-level architecture, component summary | First time understanding the system |
| 2. Package Architecture | Detailed package descriptions, dependencies | Setting up development environment |
| 3. Control Architecture | Dual control paths, controller stack | Implementing motion control |
| 4. Data Flow Architecture | Topics, actions, services, TF tree | Debugging communication issues |
| 5. Launch System Architecture | Launch strategies, timing, composition | Setting up launch files |
| 6. Robot Model Architecture | URDF/Xacro organization, joints, sensors | Modifying robot model |
| 7. Perception Architecture | Pipeline details, OctoMap integration | Working with vision/perception |
| 8. Build System Architecture | Colcon workspace, CMake patterns | Building and compiling |
| 9. Deployment Architectures | Simulation, real hardware, distributed | Deploying to different environments |
| 10. Future Extensions | Planned features, extension points | Planning new features |

### ARCHITECTURE_DIAGRAMS.md Diagrams

| Diagram | Purpose | Use Case |
|---------|---------|----------|
| 1. System Overview | Complete layer architecture | Presentations, onboarding |
| 2. Package Dependencies | How packages depend on each other | Understanding build order |
| 3. Control Architecture | Dual control path flow | Implementing controllers |
| 4. Data Flow | ROS topics and actions | Debugging communication |
| 5. Launch Hierarchy | How launch files compose | Creating new launch files |
| 6. Perception Pipeline | Vision processing steps | Perception development |
| 7. Robot Model Structure | URDF file organization | Modifying robot model |
| 8. TF Transform Tree | Coordinate frame hierarchy | TF debugging |
| 9. Controller Lifecycle | State machine for controllers | Controller management |
| 10. MoveIt Planning | Planning pipeline flow | Motion planning debugging |
| 11-12. Deployment | Simulation vs real hardware | System deployment |
| 13. Build System | Colcon build flow | Build troubleshooting |
| 14. Launch Strategies | Modular vs all-in-one | Choosing launch approach |
| 15. Timing Sequence | System startup sequence | Debugging startup issues |

---

## 🔍 Finding Information

### By Task

**I want to...**

- **Build the project** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 8, [CLAUDE.md](../CLAUDE.md) Build System
- **Launch the system** → [CLAUDE.md](../CLAUDE.md) Running the System, [ARCHITECTURE.md](ARCHITECTURE.md) Section 5
- **Understand packages** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2
- **Debug controllers** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 3, Diagram 3
- **Work with MoveIt** → [CLAUDE.md](../CLAUDE.md) Working with MoveIt, Diagram 10
- **Add perception** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 7, Diagram 6
- **Modify robot model** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 6, Diagram 7
- **Create launch file** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 5, Diagram 5
- **Deploy to hardware** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 9, Diagram 12
- **Extend system** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 10

### By Component

**I'm working on...**

- **arm_description** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, Section 6
- **arm_control** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, Section 3
- **arm_moveit_config** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, [CLAUDE.md](../CLAUDE.md)
- **arm_gazebo** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, [CLAUDE.md](../CLAUDE.md)
- **arm_perception** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, Section 7
- **arm_system_bringup** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 2.3, Section 5

### By Problem

**I'm experiencing...**

- **Controllers not spawning** → [CLAUDE.md](../CLAUDE.md) Common Issues, Diagram 15
- **MoveIt planning fails** → [CLAUDE.md](../CLAUDE.md) Common Issues, Diagram 10
- **Robot not appearing** → [CLAUDE.md](../CLAUDE.md) Common Issues, Diagram 7
- **Gazebo not starting** → [CLAUDE.md](../CLAUDE.md) Common Issues
- **Build errors** → [ARCHITECTURE.md](ARCHITECTURE.md) Section 8
- **TF errors** → Diagram 8, [ARCHITECTURE.md](ARCHITECTURE.md) Section 4.4
- **Topic communication issues** → Diagram 4, [ARCHITECTURE.md](ARCHITECTURE.md) Section 4

---

## 🚀 Quick Reference

### Essential Commands

```bash
# Build
colcon build
source install/setup.bash

# Launch system (RECOMMENDED - GUI)
ros2 run arm_gui_tools full_system_launcher.py

# Launch (command-line alternative)
ros2 launch arm_system_bringup full_system.launch.py

# Launch (legacy all-in-one with RViz)
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Status checks
ros2 control list_controllers
ros2 topic list
ros2 topic echo /joint_states

# Run examples
ros2 run arm_control example.py
```

### Key Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| controllers.yaml | Controller config | [arm_control/config/](../src/control/arm_control/config/controllers.yaml) |
| kinematics.yaml | IK solver | [arm_moveit_config/config/](../src/planning/arm_moveit_config/config/kinematics.yaml) |
| joint_limits.yaml | Planning limits | [arm_moveit_config/config/](../src/planning/arm_moveit_config/config/joint_limits.yaml) |
| arm_description.srdf | Semantic robot | [arm_moveit_config/config/](../src/planning/arm_moveit_config/config/arm_description.srdf) |
| perception.yaml | Perception params | [arm_perception/config/](../src/perception/arm_perception/config/perception.yaml) |

### Robot Specifications

- **6 DOF arm** (shoulder pitch/roll/yaw, elbow, wrist pitch/roll)
- **2 DOF gripper** (parallel jaw, 1 mimic joint)
- **1 RGBD camera** (640x480 @ 30Hz, mounted on base)
- **14 links** with visual/collision meshes
- **8 actuated joints** total

---

## 📊 Architecture Statistics

### Codebase Metrics

- **Packages:** 25 total (8 active, 17 placeholder)
- **Launch files:** 30+ across packages
- **Config files:** 14+ YAML configurations
- **Python scripts:** 30+ nodes and utilities
- **URDF/Xacro:** 8 robot model files
- **Mesh files:** 80+ STL files (visual + collision)
- **World files:** 4 Gazebo simulation environments

### ROS 2 Interfaces

- **Topics:** 15+ published (joint states, camera, TF, etc.)
- **Actions:** 2 primary (arm trajectory, gripper command)
- **Services:** 10+ (controller management, MoveIt queries)
- **TF frames:** 20+ in transform tree

### System Requirements

**Minimum:**
- 4 CPU cores
- 8 GB RAM
- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy

**Recommended:**
- 8 CPU cores
- 16 GB RAM
- GPU (for YOLO perception)
- SSD storage

---

## 🛠️ Development Workflow

### For New Developers

1. **Setup Environment**
   ```bash
   cd ~/Documents/GitHub/ldr-humanoid-arm-system
   colcon build
   source install/setup.bash
   ```

2. **Understand System**
   - Read [CLAUDE.md](../CLAUDE.md) - Project overview and GUI launcher
   - Read [ARCHITECTURE.md](ARCHITECTURE.md) Section 1-2 - Architecture
   - View [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) Diagrams 1-5

3. **Launch and Test**
   ```bash
   # Launch GUI (RECOMMENDED)
   ros2 run arm_gui_tools full_system_launcher.py

   # 1. Select world from dropdown
   # 2. Click "Start" for full system
   # 3. Optionally start Gazebo GUI, RViz, MoveIt

   # In another terminal, run example
   ros2 run arm_control example.py
   ```

4. **Verify System**
   ```bash
   ros2 control list_controllers
   ros2 topic list
   ros2 topic echo /joint_states
   ```

### For Contributors

1. **Choose a component** from [ARCHITECTURE.md](ARCHITECTURE.md) Section 10 (Future Extensions)
2. **Understand existing code** in similar active package
3. **Follow patterns** from [ARCHITECTURE.md](ARCHITECTURE.md) Section 8 (Build System)
4. **Test integration** with existing launch files
5. **Document changes** in relevant sections

---

## 📝 Maintenance

### Document Update Guidelines

**When to update ARCHITECTURE.md:**
- New package added
- Package dependencies changed
- New control mode implemented
- Configuration files modified
- Launch file hierarchy changed
- Robot model updated
- Deployment strategy changed

**When to update ARCHITECTURE_DIAGRAMS.md:**
- System architecture changed
- New data flow added
- Package dependencies modified
- Launch composition changed
- Perception pipeline updated

**When to update CLAUDE.md:**
- Quick start commands changed
- Common issues discovered
- New examples added
- Package purposes changed

### Versioning

Architecture documents follow semantic versioning:
- **Major** (1.x.x): Fundamental architecture changes
- **Minor** (x.1.x): New packages, significant features
- **Patch** (x.x.1): Documentation improvements, clarifications

---

## 🤝 Contributing

### Documentation Contributions

We welcome improvements to documentation:

1. **Clarifications** - Make existing content clearer
2. **Examples** - Add code examples and use cases
3. **Diagrams** - Add new diagrams or improve existing ones
4. **Troubleshooting** - Document solutions to common problems
5. **Tutorials** - Step-by-step guides for specific tasks

### Reporting Issues

If you find documentation issues:
1. Check if issue is already in [GitHub Issues](https://github.com/widemic/ldr-humanoid-arm-system/issues)
2. Create new issue with:
   - Document name and section
   - Description of issue
   - Suggested improvement (if applicable)

---

## 📞 Support

### Getting Help

- **Documentation unclear?** → Open a GitHub issue
- **Technical questions?** → Review [CLAUDE.md](../CLAUDE.md) Common Issues
- **Architecture questions?** → Review [ARCHITECTURE.md](ARCHITECTURE.md) relevant section
- **Visual learner?** → Start with [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md)

### Additional Resources

- **ROS 2 Jazzy Documentation:** https://docs.ros.org/en/jazzy/
- **MoveIt2 Documentation:** https://moveit.picknik.ai/
- **Gazebo Harmonic Documentation:** https://gazebosim.org/docs/harmonic
- **ros2_control Documentation:** https://control.ros.org/

---

## 📅 Changelog

### Version 1.0 (2025-11-21)

**Initial Release**
- Complete architecture documentation
- 15 Mermaid diagrams covering all aspects
- Comprehensive package descriptions
- Launch system documentation
- Perception pipeline details
- Build system specifications
- Deployment architectures
- Quick reference guides

---

**Last Updated:** 2025-11-21
**Maintainer:** LDR Robotics Team
**Repository:** https://github.com/widemic/ldr-humanoid-arm-system
