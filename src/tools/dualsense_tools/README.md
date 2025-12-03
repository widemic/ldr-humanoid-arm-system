# DualSense Tools

Simple ROS 2 package for accessing DualSense controller special features not available through standard joy plugin.

## Features

- **Battery Monitoring** - Real-time battery level publishing
- **Mute Button as Emergency Stop** - Toggle emergency stop with mute button, LED indicator shows state
- **Player LEDs for Mode** - 4 player LEDs indicate current mode (1-4)
- **Simple Interface** - Minimal complexity, easy to integrate

## Topics

### Published

- `/dualsense/battery` (std_msgs/Int8) - Battery level (0-100%)
- `/dualsense/emergency_stop` (std_msgs/Bool) - Emergency stop state
- `/dualsense/mode` (std_msgs/UInt8) - Current mode (1-4)

## Installation

### 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y libhidapi-dev libhidapi-hidraw0 libhidapi-libusb0
```

### 2. Install pydualsense

```bash
pip install pydualsense
```

### 3. Build Package

```bash
colcon build --packages-select dualsense_tools
source install/setup.bash
```

## Usage

### Run Node (with sudo)

The simplest way to run the node is with sudo:

```bash
sudo -E ros2 run dualsense_tools dualsense_node.py
```

The `-E` flag preserves your ROS environment variables.

### Alternative: Setup udev rules (optional)

If you don't want to use sudo, you can set up udev rules:

```bash
# Copy udev rules
sudo cp src/tools/dualsense_tools/99-dualsense.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconnect your controller (unplug/replug USB or disconnect/reconnect Bluetooth)
```

Then run normally:
```bash
ros2 run dualsense_tools dualsense_node.py
```

### Parameters

- `publish_rate` (default: 10.0 Hz) - Main loop rate for button checking
- `battery_check_rate` (default: 1.0 Hz) - Battery status publishing rate

### Controls

- **Mute Button** - Toggle emergency stop (LED shows state)
- **D-pad Up** - Cycle through modes 1-4 (player LEDs show current mode)

## Integration Example

Subscribe to emergency stop in your control node:

```python
from std_msgs.msg import Bool

def emergency_stop_callback(msg):
    if msg.data:
        # Stop all motion
        print("EMERGENCY STOP ACTIVE")
    else:
        # Resume operation
        print("Emergency stop released")

self.create_subscription(Bool, '/dualsense/emergency_stop', emergency_stop_callback, 10)
```

## Notes

- Controller must be connected via USB or Bluetooth before starting node
- PS button handling may vary by connection method
- Currently using D-pad Up for mode cycling (simpler than PS button)
- Mute LED color/behavior depends on controller firmware
- Only tested with DualSense (PS5 controller)

## Troubleshooting

**Controller not detected:**
- Check USB connection or Bluetooth pairing
- Verify pydualsense installation: `pip show pydualsense`
- Check permissions: May need to run with sudo or add udev rules

**LEDs not changing:**
- Some LED features require USB connection (not Bluetooth)
- Try reconnecting controller
- Check controller firmware is up to date

## Future Enhancements

If needed, could add:
- Haptic feedback for emergency stop
- Adaptive trigger effects for force feedback
- Touchpad for additional input
- RGB lightbar for status indication
