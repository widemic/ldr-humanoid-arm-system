# Power Monitoring Quick Start

Quick guide to simulate and calculate actuator power consumption.

## ⚠️ Important: Power monitoring requires joint states

The power monitor calculates consumption from `/joint_states` topic. You must either:
1. **Run the simulation** (recommended), OR
2. **Use the test script** (standalone calculations without simulation)

---

## Quick Test (No Simulation Required)

Run the test suite to see power calculations for various scenarios:

```bash
cd /home/andrei-dragomir/Documents/GitHub/ldr-humanoid-arm-system
source install/setup.bash
ros2 run arm_control test_power_monitor.py
```

This will show you:
- Power consumption in idle, slow motion, fast motion, and peak load
- Per-joint power breakdown
- Efficiency analysis
- Battery life estimates
- Optimization recommendations

**This is the easiest way to see the system working without simulation!**

## Real-Time Monitoring with Simulation

### Option 1: GUI (Recommended)

**Terminal 1 - Start simulation:**
```bash
ros2 launch arm_system_bringup full_system.launch.py
```

**Terminal 2 - Start power monitor GUI:**
```bash
ros2 run arm_gui_tools power_monitor_gui
```

**Terminal 3 - Run motion examples:**
```bash
ros2 run arm_control example.py
```

The GUI shows:
- Real-time power and current graphs
- Per-joint power bars
- Statistics (peak, average, total energy)
- Battery life estimation

### Option 2: Terminal Monitoring

**⚠️ IMPORTANT: Start simulation BEFORE power monitor, or you'll see "No joint states" warnings**

**Terminal 1 - Start simulation FIRST:**
```bash
ros2 launch arm_system_bringup full_system.launch.py
# Wait for simulation to fully start (5-10 seconds)
```

**Terminal 2 - Start power monitor node (after simulation is running):**
```bash
# Simple start
ros2 run arm_control power_monitor_node.py

# OR with launch file (recommended)
ros2 launch arm_control power_monitoring.launch.py

# OR with CSV logging
ros2 launch arm_control power_monitoring.launch.py \
  log_to_file:=true \
  log_file_path:=$HOME/power_log.csv
```

**Terminal 3 - View topics:**
```bash
# Watch total power
ros2 topic echo /power_monitor/total_power

# Watch detailed per-joint data
ros2 topic echo /power_monitor/detailed

# Watch statistics
ros2 topic echo /power_monitor/statistics
```

**Terminal 4 - Run motion:**
```bash
ros2 run arm_control example.py
```

If you see **"No joint states received. Is the simulation running?"**, it means:
- Simulation not started yet, OR
- Simulation crashed, OR
- `/joint_states` topic not being published

Check with: `ros2 topic list | grep joint_states`

## CSV Logging

Log power consumption to CSV file for later analysis:

```bash
# Using launch file (recommended)
ros2 launch arm_control power_monitoring.launch.py \
  log_to_file:=true \
  log_file_path:=$HOME/power_log.csv

# OR using ros2 run directly
ros2 run arm_control power_monitor_node.py \
  --ros-args \
  -p log_to_file:=true \
  -p log_file_path:=$HOME/power_log.csv
```

Open the CSV file in Excel/LibreOffice for analysis.

**CSV columns include:**
- Timestamp
- Total power, current, efficiency
- Per-joint current, voltage, power
- Cumulative energy

## Understanding the Results

**Typical Power Consumption:**
- **Idle:** 0-5 W
- **Slow motion:** 100-150 W
- **Fast motion:** 300-400 W
- **Peak load:** 600-800 W

**Battery Life (20Ah @ 48V):**
- Slow operation (150W avg): ~5 hours
- Normal operation (250W avg): ~3 hours
- Heavy operation (500W avg): ~1.5 hours

## How It Works

The system calculates power using motor physics:

1. **Current** = Torque / (Motor_Constant × Efficiency × Gear_Ratio)
2. **Voltage** = Back_EMF + Current × Resistance
3. **Power** = Voltage × Current

All motor parameters are in [src/control/arm_control/config/actuator_specs.yaml](src/control/arm_control/config/actuator_specs.yaml)

## Files Created

- [src/control/arm_control/scripts/power_calculator.py](src/control/arm_control/scripts/power_calculator.py) - Core calculations
- [src/control/arm_control/scripts/power_monitor_node.py](src/control/arm_control/scripts/power_monitor_node.py) - ROS 2 node
- [src/control/arm_control/scripts/test_power_monitor.py](src/control/arm_control/scripts/test_power_monitor.py) - Test suite
- [src/tools/arm_gui_tools/src/arm_gui_tools/power_monitor_gui.py](src/tools/arm_gui_tools/src/arm_gui_tools/power_monitor_gui.py) - GUI

## Documentation

Full documentation: [docs/POWER_MONITORING.md](docs/POWER_MONITORING.md)

---

**Need help?** Run `ros2 run arm_control test_power_monitor.py` for detailed examples and recommendations.
