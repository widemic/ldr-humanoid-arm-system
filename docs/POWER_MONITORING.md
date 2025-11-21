# Power Monitoring System

Complete documentation for the robot actuator power consumption monitoring system.

## Overview

The power monitoring system calculates and tracks electrical power consumption of all robot actuators in real-time. It provides:

- **Accurate current calculation** based on motor physics (torque constant, back-EMF, resistance)
- **Real-time monitoring** via ROS 2 topics
- **Graphical visualization** with PyQt5 GUI
- **Historical logging** to CSV files
- **Battery life estimation**
- **Efficiency analysis**

## Theory of Operation

### Electrical Power Calculation

The system uses fundamental motor equations to calculate power consumption:

#### 1. Motor Current
```
I = τ / (Kt × η × GR) + I_no_load
```
Where:
- `τ` = output torque at joint shaft (Nm)
- `Kt` = motor torque constant (Nm/A)
- `η` = gearbox efficiency
- `GR` = gear ratio
- `I_no_load` = no-load current (from friction)

#### 2. Motor Voltage
```
V = Ke × ω_motor + I × R
```
Where:
- `Ke` = back-EMF constant (V·s/rad)
- `ω_motor` = motor velocity before gearbox = ω_shaft × GR
- `R` = motor resistance (Ohm)

#### 3. Electrical Power
```
P_elec = V × I
```

#### 4. Mechanical Power
```
P_mech = τ × ω_shaft
```

#### 5. Efficiency
```
η = P_mech / P_elec
```

### Motor Specifications

All motor parameters are defined in [arm_control/config/actuator_specs.yaml](../src/control/arm_control/config/actuator_specs.yaml):

**ROBSTRIDE 04** (Shoulder joints):
- Rated voltage: 48V
- Rated current: 27A
- Peak current: 90A
- Torque constant (Kt): 2.1 Nm/A
- Back-EMF constant (Ke): 2.1 V·s/rad
- Motor resistance: 0.16 Ohm
- Gear ratio: 9.0
- Max torque: 120 Nm
- Brake power: 5W

**ROBSTRIDE 03** (Elbow joints):
- Rated voltage: 48V
- Rated current: 12A
- Peak current: 43A
- Torque constant (Kt): 2.36 Nm/A
- Back-EMF constant (Ke): 2.36 V·s/rad
- Motor resistance: 0.39 Ohm
- Gear ratio: 9.0
- Max torque: 60 Nm
- Brake power: 3W

**ROBSTRIDE 02** (Wrist joints):
- Rated voltage: 48V
- Rated current: 7A
- Peak current: 23A
- Torque constant (Kt): 1.22 Nm/A
- Back-EMF constant (Ke): 1.22 V·s/rad
- Motor resistance: 0.58 Ohm
- Gear ratio: 7.75
- Max torque: 17 Nm
- Brake power: 2W

## System Components

### 1. PowerCalculator (`power_calculator.py`)

Core calculation module providing:

```python
from power_calculator import PowerCalculator

calc = PowerCalculator('/path/to/actuator_specs.yaml')

# Calculate power for a single joint
current, voltage, power = calc.calculate_electrical_power(
    joint_name='left_shoulder_pitch_rs04',
    torque=30.0,  # Nm
    velocity=1.0  # rad/s
)

# Calculate total system power
joint_states = {
    'left_shoulder_pitch_rs04': {'effort': 40.0, 'velocity': 0.5},
    'left_shoulder_roll_rs04': {'effort': 35.0, 'velocity': 0.3},
    # ... other joints
}

result = calc.calculate_total_power(joint_states)
# Returns: total_power, total_current, per-joint data, efficiency
```

**Methods:**
- `calculate_motor_current(joint_name, torque, velocity)` - Calculate current draw
- `calculate_motor_voltage(joint_name, current, velocity)` - Calculate supply voltage
- `calculate_electrical_power(joint_name, torque, velocity)` - Calculate power consumption
- `calculate_mechanical_power(torque, velocity)` - Calculate mechanical output
- `calculate_efficiency(joint_name, torque, velocity)` - Calculate efficiency
- `calculate_total_power(joint_states)` - Calculate system-wide power
- `set_brake_state(joint_name, engaged)` - Control brake simulation
- `estimate_battery_life(avg_power, capacity, voltage)` - Estimate runtime

### 2. PowerMonitorNode (`power_monitor_node.py`)

ROS 2 node for real-time monitoring:

**Subscribes to:**
- `/joint_states` (sensor_msgs/JointState) - Joint positions, velocities, efforts

**Publishes to:**
- `/power_monitor/total_power` (std_msgs/Float64) - Total system power (W)
- `/power_monitor/total_current` (std_msgs/Float64) - Total current (A)
- `/power_monitor/detailed` (std_msgs/String) - Per-joint data (JSON)
- `/power_monitor/statistics` (std_msgs/String) - Statistics (JSON)

**Parameters:**
- `update_rate` (float, default: 10.0) - Publishing rate in Hz
- `actuator_specs_path` (string) - Path to actuator_specs.yaml
- `log_to_file` (bool, default: false) - Enable CSV logging
- `log_file_path` (string, default: /tmp/power_log.csv) - Log file path

**Features:**
- Real-time power calculation from joint states
- Cumulative energy tracking (Wh)
- Peak and average power statistics
- CSV logging with timestamps
- Console logging every 2 seconds

### 3. PowerMonitorGUI (`power_monitor_gui.py`)

PyQt5 graphical interface providing:

**Display Features:**
- **Total power and current** - Large display with color coding
- **Real-time graph** - Power and current vs. time (last 100 samples)
- **Per-joint bars** - Individual joint power consumption
- **Statistics panel** - Peak power, average power, total energy
- **Battery estimator** - Runtime calculation based on average power
- **Reset button** - Clear statistics and restart monitoring

**GUI Components:**
- Main power display (32px font, blue)
- Current display (24px font, orange)
- Dual-axis matplotlib graph (power in W, current in A)
- 6 progress bars for per-joint power
- Battery life calculator with configurable capacity/voltage
- Statistics: peak, average, cumulative energy, efficiency

## Usage

### Installation

1. Build the workspace:
```bash
cd /home/andrei-dragomir/Documents/GitHub/ldr-humanoid-arm-system
colcon build --packages-select arm_control arm_gui_tools
source install/setup.bash
```

2. Verify installation:
```bash
ros2 run arm_control power_calculator.py  # Test calculations
ros2 run arm_control test_power_monitor.py  # Run test suite
```

### Basic Usage

#### Method 1: Using the Test Script (Standalone)

Test the power calculator without ROS simulation:

```bash
ros2 run arm_control test_power_monitor.py
```

This runs a comprehensive test suite with:
- Idle state power
- Gravity holding power
- Slow motion power
- Fast motion power
- Peak load power
- Brake power consumption
- Efficiency analysis
- Recommendations

#### Method 2: Real-time Monitoring (with Simulation)

**Terminal 1: Start simulation**
```bash
ros2 launch arm_system_bringup full_system.launch.py
```

**Terminal 2: Start power monitor node**
```bash
ros2 run arm_control power_monitor_node.py
```

Monitor published topics:
```bash
# Total power
ros2 topic echo /power_monitor/total_power

# Total current
ros2 topic echo /power_monitor/total_current

# Detailed per-joint data
ros2 topic echo /power_monitor/detailed

# Statistics
ros2 topic echo /power_monitor/statistics
```

**Terminal 3: Run motion examples**
```bash
ros2 run arm_control example.py
```

#### Method 3: Graphical Monitoring (GUI)

**Terminal 1: Start simulation**
```bash
ros2 launch arm_system_bringup full_system.launch.py
```

**Terminal 2: Start GUI**
```bash
ros2 run arm_gui_tools power_monitor_gui
```

**Terminal 3: Run motions**
```bash
ros2 run arm_control example.py
```

The GUI will display real-time power consumption with graphs and statistics.

### CSV Logging

Enable logging to CSV file:

```bash
ros2 run arm_control power_monitor_node.py \
  --ros-args \
  -p log_to_file:=true \
  -p log_file_path:=/home/user/power_log.csv
```

CSV format:
```
timestamp, total_power_W, total_current_A, motor_power_W, brake_power_W, ...
2025-11-21T10:30:00, 245.5, 5.12, 240.5, 5.0, ...
```

### Integration with Motion Planning

Monitor power during specific trajectories:

```python
from motion_planner import MotionPlanner
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PowerAwareMotionNode(Node):
    def __init__(self):
        super().__init__('power_aware_motion')
        self.power_sub = self.create_subscription(
            Float64,
            '/power_monitor/total_power',
            self.power_callback,
            10
        )
        self.current_power = 0.0

    def power_callback(self, msg):
        self.current_power = msg.data

        # Check for excessive power
        if self.current_power > 500.0:  # 500W threshold
            self.get_logger().warn(f'High power: {self.current_power}W')

# Use in motion planning
planner = MotionPlanner()
node = PowerAwareMotionNode()

planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3, 0.0])
# Monitor power in parallel
```

## Typical Power Consumption

Based on test results from `test_power_monitor.py`:

### Idle State
- **Total Power:** ~0-5 W
- **Total Current:** ~0-0.2 A
- **Scenario:** Robot stationary, no load

### Holding Against Gravity
- **Total Power:** ~20-40 W
- **Total Current:** ~0.5-1.0 A
- **Scenario:** Arm horizontal, static position
- **Note:** Low power because velocity = 0

### Slow Motion
- **Total Power:** ~100-150 W
- **Total Current:** ~2-3 A
- **Scenario:** Coordinated motion at 0.2-0.5 rad/s
- **Efficiency:** 60-70%

### Fast Motion
- **Total Power:** ~300-400 W
- **Total Current:** ~6-8 A
- **Scenario:** Fast motion at 1.5-2.0 rad/s
- **Efficiency:** 50-60%

### Peak Load
- **Total Power:** ~600-800 W
- **Total Current:** ~12-16 A
- **Scenario:** Maximum torque + high velocity
- **Efficiency:** 40-50%
- **Note:** Not sustainable continuously

### With Brakes Engaged
- **Additional Power:** 5W (RS04) + 5W (RS04) + 3W (RS03) = 13W for all shoulder/elbow brakes
- **Use case:** Long-term static holds to save motor power

## Battery Life Estimation

Formula:
```
Runtime (hours) = (Battery_Ah × Battery_V × DOD) / Average_Power_W
```

Example calculations:

**20Ah @ 48V battery (80% DOD):**
- Idle: ~4800 hours (essentially unlimited)
- Slow motion (150W avg): 5.1 hours
- Fast motion (350W avg): 2.2 hours
- Peak load (700W avg): 1.1 hours

**40Ah @ 48V battery (80% DOD):**
- Slow motion (150W avg): 10.2 hours
- Fast motion (350W avg): 4.4 hours
- Peak load (700W avg): 2.2 hours

**Typical mixed operation (200W average):**
- 20Ah battery: 3.8 hours
- 40Ah battery: 7.7 hours

## Power Optimization Strategies

### 1. Trajectory Planning
- Use smooth acceleration profiles (jerk-limited trajectories)
- Minimize peak velocities when not needed
- Plan paths that work with gravity (downward motions are more efficient)
- Use rated speeds (17.5 rad/s) instead of max speeds (20.9 rad/s) for continuous operation

### 2. Brake Usage
- Engage electromagnetic brakes during long static holds
- Brake power (3-5W) << motor holding current power (20-50W)
- Example: Holding arm horizontal for 10 minutes
  - Without brake: 30W × 10min = 5 Wh
  - With brake: 5W × 10min = 0.83 Wh
  - Savings: 83%

### 3. Torque Limits
- Use rated torque (40 Nm for RS04) for continuous operation
- Reserve peak torque (120 Nm) for brief accelerations
- Lower torque limits in MoveIt configuration when peak performance not needed

### 4. Velocity Limits
- Default velocity limits are conservative
- For efficiency: use 50-70% of max velocity
- Higher velocities = higher currents due to back-EMF

### 5. Thermal Management
- Monitor cumulative energy (Wh) as proxy for thermal buildup
- Thermal time constant: 900s (RS04), 720s (RS03), 600s (RS02)
- Allow cooling periods after high-power operations
- Peak power should not be sustained for > 60 seconds

### 6. Load Reduction
- Minimize end-effector weight
- Optimize link design for low inertia
- Balance arm design to reduce gravity torques

## Troubleshooting

### Power readings seem too high
- Check that `actuator_specs.yaml` is loaded correctly
- Verify torque readings from `/joint_states` are reasonable
- Check for simulation artifacts (oscillations, high-frequency noise)

### Power readings seem too low
- Verify friction parameters in `actuator_specs.yaml`
- Check that velocity readings are non-zero during motion
- Ensure gearbox efficiency is set correctly (default: 0.85)

### GUI not updating
- Check that simulation is running: `ros2 topic hz /joint_states`
- Verify ROS 2 nodes can communicate: `ros2 node list`
- Check for Python errors in terminal

### CSV logging not working
- Check write permissions for log file path
- Verify `log_to_file:=true` parameter is set
- Check disk space

## Advanced Features

### Custom Power Limits

Monitor and enforce power limits:

```python
from power_calculator import PowerCalculator

calc = PowerCalculator(specs_path)

# In motion planning loop
result = calc.calculate_total_power(joint_states)

if result['total_power'] > 500.0:  # 500W limit
    # Reduce velocity scaling factor
    velocity_scale = 0.5
    # Or abort motion
    planner.stop()
```

### Thermal Modeling (Future)

The system includes thermal parameters for future thermal modeling:
- `thermal_time_constant` - Cooling time constant (seconds)
- `max_temperature` - Maximum safe temperature (°C)

Future implementation could track:
```python
# Heat generation rate
Q_gen = P_elec - P_mech  # Losses -> heat

# Temperature rise
dT/dt = Q_gen / (m × c) - (T - T_ambient) / τ

# Where:
# m = motor mass
# c = specific heat
# τ = thermal_time_constant
```

### Power-Aware Motion Planning (Future)

Integrate with MoveIt2 for power-optimal trajectories:
- Add power cost function to OMPL planner
- Minimize ∫ P(t) dt over trajectory
- Trade-off: execution time vs. energy consumption

## References

- Motor specifications: [actuator_specs.yaml](../src/control/arm_control/config/actuator_specs.yaml)
- ROBSTRIDE motor documentation: (manufacturer datasheets)
- ROS 2 joint_states message: `ros2 interface show sensor_msgs/msg/JointState`

## Files

```
src/control/arm_control/
├── config/
│   └── actuator_specs.yaml         # Motor specifications (source of truth)
└── scripts/
    ├── power_calculator.py         # Core calculation module
    ├── power_monitor_node.py       # ROS 2 monitoring node
    └── test_power_monitor.py       # Test suite and examples

src/tools/arm_gui_tools/
└── src/arm_gui_tools/
    └── power_monitor_gui.py        # PyQt5 GUI

docs/
└── POWER_MONITORING.md             # This file
```

## Future Improvements

1. **Custom ROS message types** - Replace String messages with custom msg definitions
2. **Thermal modeling** - Track motor temperature based on power dissipation
3. **Power-optimal planning** - Integrate with MoveIt2 cost functions
4. **Battery state tracking** - Monitor actual battery voltage/current via hardware interface
5. **Historical analysis** - Tools for analyzing logged CSV data
6. **Alerts and warnings** - Configurable thresholds for power/current/energy
7. **Multi-robot support** - Monitor multiple arms simultaneously

---

**Last Updated:** 2025-11-21
**Author:** Claude Code
**Version:** 1.0
