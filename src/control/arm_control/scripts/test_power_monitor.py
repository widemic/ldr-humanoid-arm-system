#!/usr/bin/env python3
"""
Test script for power monitoring system.

This script:
1. Tests the PowerCalculator with example joint states
2. Demonstrates power calculation for various motion scenarios
3. Provides recommendations for power optimization
"""

import os
import sys
import time
from power_calculator import PowerCalculator


def print_header(text):
    """Print formatted section header."""
    print("\n" + "=" * 80)
    print(f"  {text}")
    print("=" * 80)


def print_power_result(result, scenario_name):
    """Print formatted power calculation result."""
    print(f"\n{scenario_name}:")
    print(f"  Total Power:       {result['total_power']:>8.2f} W")
    print(f"  Total Current:     {result['total_current']:>8.2f} A")
    print(f"  Motor Power:       {result['motor_power']:>8.2f} W")
    print(f"  Mechanical Power:  {result['mechanical_power']:>8.2f} W")
    print(f"  System Efficiency: {result['efficiency']*100:>7.1f} %")


def test_idle_state(calc):
    """Test power consumption in idle state."""
    print_header("TEST 1: Idle State (No Motion)")

    joint_states = {
        'shoulder_pitch_joint': {'effort': 0.0, 'velocity': 0.0},
        'shoulder_roll_joint': {'effort': 0.0, 'velocity': 0.0},
        'shoulder_yaw_joint': {'effort': 0.0, 'velocity': 0.0},
        'elbow_pitch_joint': {'effort': 0.0, 'velocity': 0.0},
        'elbow_yaw_joint': {'effort': 0.0, 'velocity': 0.0},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Idle (All joints stationary)")


def test_gravity_holding(calc):
    """Test power consumption when holding against gravity."""
    print_header("TEST 2: Holding Against Gravity")

    # Simulate holding arm horizontal (significant torque, no velocity)
    joint_states = {
        'shoulder_pitch_joint': {'effort': 40.0, 'velocity': 0.0},  # Holding up
        'shoulder_roll_joint': {'effort': 30.0, 'velocity': 0.0},   # Side load
        'shoulder_yaw_joint': {'effort': 5.0, 'velocity': 0.0},
        'elbow_pitch_joint': {'effort': 15.0, 'velocity': 0.0},
        'elbow_yaw_joint': {'effort': 3.0, 'velocity': 0.0},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Holding horizontal position")

    print("\n  Note: Power consumption is low because velocity is zero.")
    print("  Current is needed only to overcome friction and maintain position.")


def test_slow_motion(calc):
    """Test power consumption during slow motion."""
    print_header("TEST 3: Slow Motion")

    joint_states = {
        'shoulder_pitch_joint': {'effort': 30.0, 'velocity': 0.3},
        'shoulder_roll_joint': {'effort': 25.0, 'velocity': 0.2},
        'shoulder_yaw_joint': {'effort': 10.0, 'velocity': 0.5},
        'elbow_pitch_joint': {'effort': 15.0, 'velocity': 0.4},
        'elbow_yaw_joint': {'effort': 5.0, 'velocity': 0.6},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Slow coordinated motion")


def test_fast_motion(calc):
    """Test power consumption during fast motion."""
    print_header("TEST 4: Fast Motion")

    joint_states = {
        'shoulder_pitch_joint': {'effort': 60.0, 'velocity': 1.5},
        'shoulder_roll_joint': {'effort': 50.0, 'velocity': 1.2},
        'shoulder_yaw_joint': {'effort': 25.0, 'velocity': 1.8},
        'elbow_pitch_joint': {'effort': 30.0, 'velocity': 2.0},
        'elbow_yaw_joint': {'effort': 10.0, 'velocity': 2.5},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Fast coordinated motion")

    print("\n  Per-Joint Breakdown:")
    for joint, data in result['joints'].items():
        print(f"    {joint}:")
        print(f"      Current: {data['current']:>6.2f} A")
        print(f"      Voltage: {data['voltage']:>6.2f} V")
        print(f"      Power:   {data['power']:>6.2f} W")


def test_peak_load(calc):
    """Test power consumption at peak load."""
    print_header("TEST 5: Peak Load Scenario")

    # Maximum torque at moderate velocity
    joint_states = {
        'shoulder_pitch_joint': {'effort': 100.0, 'velocity': 1.0},
        'shoulder_roll_joint': {'effort': 90.0, 'velocity': 0.8},
        'shoulder_yaw_joint': {'effort': 45.0, 'velocity': 1.5},
        'elbow_pitch_joint': {'effort': 50.0, 'velocity': 1.8},
        'elbow_yaw_joint': {'effort': 15.0, 'velocity': 2.0},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Peak load (heavy object manipulation)")

    # Battery life estimation
    battery_ah = 20.0
    runtime = calc.estimate_battery_life(result['total_power'], battery_ah)
    print(f"\n  Battery Life Estimate (20Ah @ 48V):")
    print(f"    Runtime: {runtime:.2f} hours ({runtime*60:.0f} minutes)")


def test_brake_power(calc):
    """Test brake power consumption."""
    print_header("TEST 6: Brake Power Consumption")

    # Engage brakes on shoulder joints
    calc.set_brake_state('shoulder_pitch_joint', True)
    calc.set_brake_state('shoulder_roll_joint', True)

    joint_states = {
        'shoulder_pitch_joint': {'effort': 0.0, 'velocity': 0.0},
        'shoulder_roll_joint': {'effort': 0.0, 'velocity': 0.0},
        'shoulder_yaw_joint': {'effort': 0.0, 'velocity': 0.0},
        'elbow_pitch_joint': {'effort': 0.0, 'velocity': 0.0},
        'elbow_yaw_joint': {'effort': 0.0, 'velocity': 0.0},
    }

    result = calc.calculate_total_power(joint_states)
    print_power_result(result, "Idle with brakes engaged")
    print(f"  Brake Power:       {result['brake_power']:>8.2f} W")

    # Reset brakes
    calc.set_brake_state('shoulder_pitch_joint', False)
    calc.set_brake_state('shoulder_roll_joint', False)


def test_efficiency_analysis(calc):
    """Analyze efficiency at different operating points."""
    print_header("TEST 7: Efficiency Analysis")

    print("\nEfficiency vs. Load:")
    print(f"{'Torque (Nm)':<15} {'Velocity (rad/s)':<20} {'Efficiency (%)':<15}")
    print("-" * 50)

    joint = 'shoulder_pitch_joint'

    test_points = [
        (10.0, 0.5),
        (20.0, 0.8),
        (40.0, 1.0),
        (60.0, 1.5),
        (80.0, 1.8),
        (100.0, 2.0),
    ]

    for torque, velocity in test_points:
        eff = calc.calculate_efficiency(joint, torque, velocity)
        print(f"{torque:<15.1f} {velocity:<20.1f} {eff*100:<15.1f}")

    print("\n  Observation: Efficiency typically decreases at very high loads.")


def print_recommendations():
    """Print power optimization recommendations."""
    print_header("Power Optimization Recommendations")

    recommendations = [
        "1. MOTION PLANNING:",
        "   - Use smooth trajectories to minimize acceleration (reduces peak current)",
        "   - Plan paths that work with gravity when possible",
        "   - Avoid unnecessary fast motions unless required",
        "",
        "2. IDLE POWER:",
        "   - Engage electromagnetic brakes when holding static positions",
        "   - This saves motor power during long holds",
        "   - Brakes consume only 5W (RS04) or 3W (RS03) vs. motor idle current",
        "",
        "3. TRAJECTORY OPTIMIZATION:",
        "   - Minimize velocity and acceleration limits when not needed",
        "   - Use rated speeds (vs. max speeds) for continuous operation",
        "   - Consider operating at rated torque (40Nm vs 120Nm for RS04)",
        "",
        "4. THERMAL MANAGEMENT:",
        "   - Monitor cumulative energy (thermal buildup)",
        "   - Allow cooling periods after high-power operations",
        "   - Peak power should not be sustained continuously",
        "",
        "5. BATTERY SIZING:",
        "   - Size battery based on average power, not peak",
        "   - For typical operation (~200W avg): 20Ah @ 48V = 4.8 hours",
        "   - Add 20% margin for thermal losses and aging",
        "",
        "6. REAL-TIME MONITORING:",
        "   - Use power_monitor_node.py for continuous logging",
        "   - Use power_monitor_gui.py for visualization",
        "   - Set alerts for excessive power consumption"
    ]

    for line in recommendations:
        print(line)


def main():
    """Main test function."""
    print("\n" + "#" * 80)
    print("#" + " " * 78 + "#")
    print("#" + "  ROBOT ACTUATOR POWER CONSUMPTION TEST SUITE".center(78) + "#")
    print("#" + " " * 78 + "#")
    print("#" * 80)

    # Load power calculator
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('arm_control')
        specs_path = os.path.join(pkg_share, 'config', 'actuator_specs.yaml')
    except:
        # Fallback for development
        specs_path = '/home/andrei-dragomir/Documents/GitHub/ldr-humanoid-arm-system/src/control/arm_control/config/actuator_specs.yaml'

    print(f"\nLoading actuator specs from: {specs_path}")

    calc = PowerCalculator(specs_path)

    # Run tests
    test_idle_state(calc)
    test_gravity_holding(calc)
    test_slow_motion(calc)
    test_fast_motion(calc)
    test_peak_load(calc)
    test_brake_power(calc)
    test_efficiency_analysis(calc)

    # Print recommendations
    print_recommendations()

    # Summary
    print_header("SUMMARY")
    print("""
This power monitoring system provides:

1. **power_calculator.py** - Core calculation module
   - Motor current calculation based on torque and Kt
   - Voltage calculation from back-EMF (Ke) and resistance
   - Efficiency analysis
   - Battery life estimation

2. **power_monitor_node.py** - ROS 2 monitoring node
   - Real-time power monitoring from /joint_states
   - Publishing to /power_monitor/* topics
   - CSV logging for analysis
   - Statistics tracking (peak, average, cumulative energy)

3. **power_monitor_gui.py** - PyQt5 visualization
   - Real-time power and current graphs
   - Per-joint power bars
   - Battery life estimation
   - Statistics display

USAGE:
------
# Start simulation
ros2 launch arm_system_bringup full_system.launch.py

# Start power monitor node (terminal logging)
ros2 run arm_control power_monitor_node.py

# Start GUI (graphical display)
ros2 run arm_gui_tools power_monitor_gui.py

# Run motion examples
ros2 run arm_control example.py

# View published topics
ros2 topic echo /power_monitor/total_power
ros2 topic echo /power_monitor/detailed
    """)

    print("\n" + "#" * 80)
    print("  Tests completed successfully!")
    print("#" * 80 + "\n")


if __name__ == '__main__':
    main()
