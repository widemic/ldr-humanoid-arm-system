# arm_teleop

Teleoperation utilities (keyboard, joystick, and GUI) for the LDR humanoid arm.

## Features
- Incremental jogging for all six arm joints via keyboard, joystick, or GUI
- Quick access to the predefined `home`, `ready`, and `vertical` poses
- Configurable jogging step size, joystick axis mapping, and trajectory durations
- Direct communication with `/arm_controller/follow_joint_trajectory`

## Core Teleop Node

Start the core teleop node before launching any client. The
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
| Base up/down (`left_shoulder_pitch_rs04`) | `q` | `a` |
| Shoulder forward/back (`left_shoulder_roll_rs04`) | `w` | `s` |
| Shoulder rotate out/in (`left_shoulder_yaw_rs03`) | `e` | `d` |
| Elbow bend/extend (`left_elbow_rs03`) | `r` | `f` |
| Wrist up/down (`left_wrist_rs02`) | `t` | `g` |
| Hand rotate (`left_hand_rs02`) | `y` | `u` |

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

## Joystick Client

The joystick client consumes `sensor_msgs/msg/Joy` messages (e.g., from
`ros2 run joy joy_node`) and continuously jogs each arm joint according to the
configured axis bindings.

```bash
source install/setup.bash
ros2 run joy joy_node            # start the joystick driver (if not already running)
ros2 run arm_teleop joystick_teleop.py --ros-args -p joy_topic:=/joy
# or launch everything together:
ros2 launch arm_teleop joystick_teleop.launch.py controller_profile:=dualsense
```

Default bindings (Xbox-style layout):

| Joint | Axis | Notes |
| ----- | ---- | ----- |
| Base up/down (`left_shoulder_pitch_rs04`) | Axis 1 (left stick vertical) | Positive pulls the arm up |
| Shoulder forward/back (`left_shoulder_roll_rs04`) | Axis 0 (left stick horizontal) | Positive moves forward |
| Shoulder rotate out/in (`left_shoulder_yaw_rs03`) | Axis 3 (right stick horizontal) | Positive rotates outward |
| Elbow bend/extend (`left_elbow_rs03`) | Axis 4 (right stick vertical) | Positive bends the elbow |
| Wrist up/down (`left_wrist_rs02`) | Axis 2 | Requires a centered trigger |
| Hand rotate (`left_hand_rs02`) | Axis 5 | Requires a centered trigger |

Buttons:
- `A` (0): hold position (re-send the target)
- `Y` (3): capture the current joint state
- `LB` (4) / `RB` (5) / `B` (1): move to `home`, `ready`, or `vertical` poses

DualSense profile:

| Joint | Axis | Notes |
| ----- | ---- | ----- |
| Base up/down | Axis 1 (left stick vertical) | Matches Sony layout |
| Shoulder forward/back | Axis 0 (left stick horizontal) | Positive moves forward |
| Shoulder rotate out/in | Axis 2 (right stick horizontal) | Positive rotates outward |
| Elbow bend/extend | Axis 3 (right stick vertical) | Positive bends the elbow |
| Wrist up/down | Axis 4 (L2 trigger) | Trigger must be centered |
| Hand rotate | Axis 5 (R2 trigger) | Trigger must be centered |

Buttons remain the same as the Xbox profile (`Cross`=0 for hold, `Triangle`=3 for capture, `L1`/`R1`/`Circle` for preset poses).
Enable this mapping via `--ros-args -p controller_profile:=dualsense`.

Parameters (overridable with `--ros-args -p ...`):
- `controller_profile` (default `dualsense`): quick axis/button presets; options include `dualsense` and `xbox`. Custom `axis_*` parameters override profile defaults.
- `joy_device`, `joy_deadzone`, `joy_autorepeat` (launch only): exposed via `joystick_teleop.launch.py` to configure the `joy_node` driver.
- `joy_topic` (default `/joy`): joystick topic to subscribe to.
- `update_rate` (default `30.0`): integration rate (Hz) used for jogging.
- `deadzone` (default `0.15`): ignore axis values whose absolute magnitude is below this threshold.
- `axis_bindings`: integer array mapping each joint to an axis index (length must match the joint count).
- `axis_scales`: float array defining the max radians/second per joint.
- `capture_button`, `resend_button`, `preset_buttons.home`, `preset_buttons.ready`, `preset_buttons.vertical`: customize button indices.
- `latched_joint_states` (default `False`): set to `True` only if your `/joint_states` publisher uses transient-local durability.
