# Joint Monitor GUI

Simple PyQt5 GUI for monitoring current joint positions using progress bars.

## Files

- **ui/joint_monitor.ui** - Qt Designer UI file with 5 progress bars for each joint
- **src/arm_gui_tools/joint_monitor.py** - Python script that loads the UI and subscribes to `/joint_states`

## Requirements

Install PyQt5:
```bash
pip install PyQt5
```

## Building

```bash
# From repository root
colcon build --packages-select arm_gui_tools
source install/setup.bash
```

## Usage

### Terminal 1: Launch the simulation
```bash
ros2 launch arm_control sim.launch.py
```

### Terminal 2: Launch the joint monitor GUI
```bash
ros2 run arm_gui_tools joint_monitor
```

The GUI will display 5 progress bars showing real-time joint positions:
- Base Rotation (-3.14 to 3.14 rad)
- Shoulder Pitch (-0.55 to 3.1 rad)
- Elbow Pitch (-3.14 to 3.14 rad)
- Wrist Pitch (-0.31 to 2.8 rad)
- Wrist Roll (-3.14 to 3.14 rad)

## Customization

To modify the UI:
1. Open `ui/joint_monitor.ui` in Qt Designer
2. Edit the layout, colors, or add new widgets
3. Rebuild the package

The progress bars are scaled by 100 internally to maintain precision (e.g., -3.14 rad = -314 on the bar).
