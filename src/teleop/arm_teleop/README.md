# arm_teleop

Keyboard teleoperation utilities for the LDR humanoid arm.

## Features
- Incremental jogging for all six arm joints using a simple key layout
- Quick access to the predefined `home`, `ready`, and `vertical` poses
- Configurable joint step size and trajectory durations
- Direct communication with `/arm_controller/follow_joint_trajectory`

## Core Teleop Node

Start the core teleop node before launching any client (keyboard or GUI). The
C++ node (`joint_teleop_node`) subscribes to `/joint_states`, listens for joint
commands on `teleop/joint_commands`, and sends trajectories to
`/arm_controller/follow_joint_trajectory`.

```bash
source install/setup.bash
ros2 run arm_teleop joint_teleop_node
```

## Keyboard Client

> **Note:** The keyboard client requires a TTY so it must be run from a normal
> terminal (or a multiplexer pane) using `ros2 run`. It publishes JointState
> commands on `teleop/joint_commands`.

```bash
source install/setup.bash
ros2 run arm_teleop keyboard_teleop.py
```

Key bindings:

| Motion (joint) | Increase | Decrease |
| -------------- | -------- | -------- |
| Base up/down (`left_shoulder_pitch_rs04_joint`) | `q` | `a` |
| Shoulder forward/back (`left_shoulder_roll_rs04_joint`) | `w` | `s` |
| Shoulder rotate out/in (`left_shoulder_yaw_rs03_joint`) | `e` | `d` |
| Elbow bend/extend (`left_elbow_rs03_joint`) | `r` | `f` |
| Wrist up/down (`left_wrist_rs02_joint`) | `t` | `g` |
| Hand rotate (`left_hand_rs02_joint`) | `y` | `u` |

Additional shortcuts:
- `1` / `2` / `3`: move to `home`, `ready`, or `vertical`.
- `c`: capture the current joint state as the teleop target.
- `space`: resend the most recent target to hold position.
- `h` or `?`: print the help banner.
- `Ctrl+C` or `Esc`: quit.

## Parameters
- `step` (float, default `0.05`): joint increment (rad) per key press.

## Launch

A convenience launch file is available when you need to start the teleop node and
keyboard client together:

```bash
ros2 launch arm_teleop keyboard_teleop.launch.py
```

Make sure the launch system keeps the keyboard teleop process attached to a TTY.

## PyQt5 GUI

A desktop GUI equivalent is available if you prefer buttons over the keyboard. It
exposes the same jogging controls, preset poses, and live joint feedback. Make
sure `joint_teleop_node` is running before launching the GUI.

```bash
ros2 run arm_teleop gui_teleop.py
```

The GUI relies on PyQt5, so it needs an environment with an available display (or
a forwarded X/Wayland session). Like the keyboard client, it publishes JointState
commands on `teleop/joint_commands` and listens to `/joint_states` for feedback.
