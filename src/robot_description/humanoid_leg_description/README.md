# Robot Description Package

This package contains the URDF description for the KBOT humanoid robot (legs, torso, and head).

## Contents

- **urdf/**: URDF/Xacro files for robot description
  - `robot.urdf.xacro`: Main robot file
  - `links/`: Link definitions (torso, legs)
  - `joints/`: Joint definitions
  - `macros/`: Reusable macros (materials, Gazebo, ros2_control)

- **meshes/**: STL mesh files for visualization and collision

- **worlds/**: Gazebo world files
  - `empty.sdf`: Simple empty world with ground plane

- **launch/**: Launch files
  - `display.launch.py`: View robot in RViz
  - `gazebo.launch.py`: Full Gazebo simulation with robot spawn
  - `spawn_robot.launch.py`: Spawn robot in existing Gazebo

## Usage

### View in RViz

```bash
ros2 launch robot_description display.launch.py
```

### Simulate in Gazebo

**Full simulation (starts Gazebo + spawns robot):**
```bash
ros2 launch robot_description gazebo.launch.py
```

**With custom world:**
```bash
ros2 launch robot_description gazebo.launch.py world:=empty.sdf
```

**Headless mode (no GUI):**
```bash
ros2 launch robot_description gazebo.launch.py gui:=false headless:=true
```

### Spawn robot in existing Gazebo

First, start Gazebo separately:
```bash
gz sim empty.sdf
```

Then spawn the robot:
```bash
ros2 launch robot_description spawn_robot.launch.py
```

**Spawn at custom position:**
```bash
ros2 launch robot_description spawn_robot.launch.py x:=1.0 y:=2.0 z:=0.0
```

## Launch Arguments

### gazebo.launch.py
- `world`: World file name (default: `empty.sdf`)
- `use_sim_time`: Use simulation clock (default: `true`)
- `gui`: Start Gazebo GUI (default: `true`)
- `headless`: Run headless (default: `false`)
- `use_sim`: Use sim hardware interface (default: `true`)
- `prefix`: Robot link/joint prefix (default: `''`)

### spawn_robot.launch.py
- `use_sim_time`: Use simulation clock (default: `true`)
- `use_sim`: Use sim hardware interface (default: `true`)
- `prefix`: Robot link/joint prefix (default: `''`)
- `x`, `y`, `z`: Spawn position (default: `0.0`)
- `roll`, `pitch`, `yaw`: Spawn orientation (default: `0.0`)

## Robot Specifications

**Links:**
- base_link (floating base)
- torso_link
- imu_link
- Left leg: hip_yoke, hip_roll, hip_pitch, femur, shin, foot
- Right leg: hip_yoke, hip_roll, hip_pitch, femur, shin, foot

**Joints:**
- Left leg: hip_pitch, hip_roll, hip_yaw, knee, ankle, foot
- Right leg: hip_pitch, hip_roll, hip_yaw, knee, ankle, foot

**Total height:** ~0.813m (from ground to torso)

## Dependencies

- ros2 (Jazzy)
- gazebo_harmonic
- ros_gz_sim
- ros_gz_bridge
- robot_state_publisher
- xacro
