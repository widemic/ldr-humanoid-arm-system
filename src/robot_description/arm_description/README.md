# arm_description

URDF description and 3D meshes for the 5-DOF humanoid arm system.

## Overview

This package contains the complete robot description for the humanoid arm, including:
- URDF model with full kinematic chain
- High-quality STL meshes for visualization and collision detection
- RViz launch files for robot visualization
- Inertial properties for dynamics simulation

## Robot Specifications

### Degrees of Freedom (DOF): 5

| Joint | Type | Range | Max Effort | Max Velocity |
|-------|------|-------|------------|--------------|
| **base_rotation_joint** (roll) | Revolute | ±180° (±3.14 rad) | 120 Nm | 3.0 rad/s |
| **shoulder_pitch_joint** | Revolute | -31.5° to 177.6° (-0.55 to 3.1 rad) | 120 Nm | 3.0 rad/s |
| **elbow_pitch_joint** | Revolute | ±180° (±3.14 rad) | 60 Nm | 1.6 rad/s |
| **wrist_pitch_joint** | Revolute | -17.8° to 160.4° (-0.31 to 2.8 rad) | 60 Nm | 1.6 rad/s |
| **wrist_roll_joint** | Revolute | ±180° (±3.14 rad) | 17 Nm | 0.8 rad/s |

### Physical Properties

- **Total Links:** 14 (5 revolute joints + 8 fixed mounting joints + 1 base)
- **Base Mass:** 400 kg (support structure)
- **Moving Mass:** ~4.1 kg (all articulated components)
- **Mesh Files:** 28 STL files (14 visual + 14 collision)
- **Total Mesh Size:** ~4.6 MB

### Link Structure

```
base_link (400 kg support)
└── shoulder_motor_link (0.53 kg)
    └── upper_arm_link (1.20 kg)
        └── shoulder_pitch_motor_link (0.53 kg)
            └── forearm_link (0.97 kg)
                └── elbow_motor_link (0.36 kg)
                    └── elbow_flange_link (0.036 kg)
                        └── wrist_base_link (0.49 kg)
                            └── wrist_motor_link (0.36 kg)
                                └── wrist_pitch_link (0.71 kg)
                                    └── wrist_assembly_link (0.25 kg)
                                        └── wrist_roll_motor_link (0.18 kg)
                                            └── end_effector_link (0.27 kg)
                                                └── end_motor_link (0.18 kg)
```

## Package Contents

```
arm_description/
├── config/                    # RViz configuration files
│   └── view_arm.rviz         # (optional) Saved RViz settings
├── launch/                    # Launch files
│   └── view_arm.launch.py    # Visualize robot in RViz
├── meshes/                    # 3D mesh files
│   ├── collision/            # 14 collision meshes (STL)
│   └── visual/               # 14 visual meshes (STL)
├── urdf/                      # Robot description
│   └── humanoid_arm_5dof.urdf # Complete URDF model (717 lines)
├── CMakeLists.txt            # Build configuration
├── package.xml               # Package metadata
└── README.md                 # This file
```

## Dependencies

This package depends on:
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher` - Publishes joint states
- `joint_state_publisher_gui` - Interactive joint control
- `rviz2` - 3D visualization tool
- `urdf` - URDF parsing library
- `xacro` - URDF macro processing (future use)

## Building

### Build this package only:
```bash
cd /path/to/ldr-humanoid-arm-system
colcon build --packages-select arm_description
source install/setup.bash
```

### Build with dependencies:
```bash
colcon build --packages-up-to arm_description
source install/setup.bash
```

## Usage

### 1. Basic Visualization with Joint Sliders

Launch RViz with interactive joint control:

```bash
ros2 launch arm_description view_arm.launch.py
```

This will open:
- **RViz2** window showing the 3D robot model
- **Joint State Publisher GUI** with sliders to control each joint

**First Time Setup in RViz:**
1. Click **Add** button (bottom left)
2. Select **RobotModel** → Click **OK**
3. (Optional) Add **TF** display to see coordinate frames
4. (Optional) Add **Axes** display for reference
5. Set **Fixed Frame** to `base_link` (top left, Global Options)
6. Adjust view with mouse:
   - Left-click + drag: Rotate
   - Middle-click + drag: Pan
   - Scroll wheel: Zoom

**Save Configuration:**
```
File → Save Config As → config/view_arm.rviz
```

### 2. Visualization Without Joint GUI

Launch with fixed joint positions:

```bash
ros2 launch arm_description view_arm.launch.py gui:=false
```

### 3. With Saved RViz Configuration

After saving an RViz config file:

```bash
ros2 launch arm_description view_arm.launch.py use_rviz_config:=true
```

### 4. For Simulation (Gazebo)

When using with simulation time:

```bash
ros2 launch arm_description view_arm.launch.py use_sim_time:=true
```

## Launch File Arguments

The `view_arm.launch.py` file supports the following arguments:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `gui` | bool | `true` | Enable joint_state_publisher_gui sliders |
| `use_sim_time` | bool | `false` | Use simulation clock instead of system clock |
| `use_rviz_config` | bool | `false` | Load saved RViz configuration from config/ |

### Examples

```bash
# Visualization with all defaults (GUI enabled)
ros2 launch arm_description view_arm.launch.py

# No joint sliders, manual joint states only
ros2 launch arm_description view_arm.launch.py gui:=false

# With simulation time and saved config
ros2 launch arm_description view_arm.launch.py use_sim_time:=true use_rviz_config:=true

# All arguments specified
ros2 launch arm_description view_arm.launch.py gui:=true use_sim_time:=false use_rviz_config:=false
```

## ROS 2 Topics

When running `view_arm.launch.py`, the following topics are available:

### Published Topics

- `/joint_states` (sensor_msgs/JointState)
  - Current joint positions, velocities, and efforts
  - Published by: joint_state_publisher or joint_state_publisher_gui

- `/tf` (tf2_msgs/TFMessage)
  - Transform tree from base_link to all robot links
  - Published by: robot_state_publisher

- `/tf_static` (tf2_msgs/TFMessage)
  - Static transforms (fixed joints)
  - Published by: robot_state_publisher

### Subscribed Topics

- `/joint_states` (sensor_msgs/JointState)
  - Robot state publisher listens for joint updates
  - Consumed by: robot_state_publisher

## Inspecting the URDF

### View URDF structure:
```bash
ros2 run xacro xacro $(ros2 pkg prefix arm_description)/share/arm_description/urdf/humanoid_arm_5dof.urdf
```

### Check URDF validity:
```bash
check_urdf $(ros2 pkg prefix arm_description)/share/arm_description/urdf/humanoid_arm_5dof.urdf
```

### Generate URDF graph:
```bash
urdf_to_graphiz $(ros2 pkg prefix arm_description)/share/arm_description/urdf/humanoid_arm_5dof.urdf
```

## Coordinate Frames (TF Tree)

```
base_link
└── shoulder_motor_link
    └── upper_arm_link
        └── shoulder_pitch_motor_link
            └── forearm_link
                └── elbow_motor_link
                    └── elbow_flange_link
                        └── wrist_base_link
                            └── wrist_motor_link
                                └── wrist_pitch_link
                                    └── wrist_assembly_link
                                        └── wrist_roll_motor_link
                                            └── end_effector_link
                                                └── end_motor_link
```

View live TF tree:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Or use:
```bash
ros2 run tf2_tools view_frames
```

## Mesh File Details

All meshes are in STL format with the following naming convention:

| Mesh File | Component | Size |
|-----------|-----------|------|
| `suport.STL` | Base support structure | ~400 KB |
| `pmotor1.STL` | Shoulder motor housing | 25 KB |
| `pmotor2.STL` | Shoulder pitch motor | 25 KB |
| `pmotor3.STL` | Elbow motor housing | 77 KB |
| `pmotor4.STL` | Wrist motor housing | 77 KB |
| `pmotor5.STL` | Wrist roll motor | 40 KB |
| `pmotor6.STL` | End effector motor | 40 KB |
| `umar1.STL` | Upper arm segment | ~200 KB |
| `umar2.STL` | Forearm segment | ~250 KB |
| `flansa.STL` | Elbow flange | 121 KB |
| `cot1.STL` | Wrist base | 392 KB |
| `cot2.STL` | Wrist pitch link | 226 KB |
| `cot3.STL` | Wrist assembly | 208 KB |
| `incheietura.STL` | End effector wrist | 288 KB |

**Color Scheme:** Light blue-purple (RGB: 202, 209, 238)

## Integration with Other Packages

This package is used by:
- **arm_control** - Controller configuration
- **arm_hardware** - Hardware interface
- **arm_moveit_config** - Motion planning setup
- **arm_gazebo** - Simulation environment
- **arm_demos** - Demonstration applications
- **arm_system_bringup** - System orchestration

## Troubleshooting

### Issue: RViz shows no robot model

**Solution:**
1. Check that RobotModel display is added
2. Verify "Robot Description" parameter is set to `/robot_description`
3. Confirm Fixed Frame is set to `base_link`
4. Check terminal for error messages

### Issue: Joint sliders not appearing

**Solution:**
1. Verify `joint_state_publisher_gui` is installed:
   ```bash
   sudo apt install ros-jazzy-joint-state-publisher-gui
   ```
2. Check that `gui:=true` is set (default)

### Issue: Mesh files not loading

**Solution:**
1. Verify package is built and sourced:
   ```bash
   colcon build --packages-select arm_description
   source install/setup.bash
   ```
2. Check mesh paths in URDF use `package://` URIs
3. Confirm meshes exist in install directory:
   ```bash
   ls $(ros2 pkg prefix arm_description)/share/arm_description/meshes/
   ```

### Issue: "Package not found" error

**Solution:**
```bash
# Source the workspace
source install/setup.bash

# Verify package is visible
ros2 pkg list | grep arm_description

# Rebuild if necessary
colcon build --packages-select arm_description --symlink-install
```

## Future Enhancements

- [ ] Convert URDF to Xacro for parameterization
- [ ] Add Gazebo-specific tags for simulation
- [ ] Create simplified collision meshes for better performance
- [ ] Add sensor mounting points (camera, force-torque sensor)
- [ ] Include end-effector attachment interface
- [ ] Add joint transmission specifications for ros2_control

## License

MIT License - See main repository LICENSE file

## Maintainer

Your Name <your.email@example.com>

## Version

0.1.0 - Initial release with complete 5-DOF arm description

## Related Packages

- [arm_control](../../../control/arm_control/) - Controller configuration
- [arm_moveit_config](../../../planning/arm_moveit_config/) - Motion planning
- [arm_gazebo](../../../simulation/arm_gazebo/) - Simulation
- [arm_demos](../../../applications/arm_demos/) - Example applications

## References

- [ROS 2 URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)
- [joint_state_publisher](https://github.com/ros/joint_state_publisher)
- [RViz2 User Guide](https://github.com/ros2/rviz)
