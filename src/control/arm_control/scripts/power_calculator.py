#!/usr/bin/env python3
"""
Power and current consumption calculator for robot actuators.

This module calculates electrical power consumption based on:
- Motor torque and velocity (mechanical power)
- Motor electrical constants (Kt, Ke, R)
- Gearbox efficiency
- Brake power consumption

Theory:
-------
For a brushless DC motor with gearbox:

1. Motor current (simplified): I = τ / (Kt * η * GR) + I_no_load
   Where:
   - τ = output torque at shaft (after gearbox)
   - Kt = motor torque constant (Nm/A)
   - η = gearbox efficiency
   - GR = gear ratio
   - I_no_load = no-load current (estimated from friction)

2. Motor voltage: V = Ke * ω_motor + I * R
   Where:
   - Ke = back-EMF constant (V·s/rad)
   - ω_motor = motor velocity (before gearbox) = ω_shaft * GR
   - R = motor resistance (Ohm)

3. Electrical power: P_elec = V * I

4. Mechanical power: P_mech = τ * ω_shaft

5. Losses: P_loss = P_elec - P_mech
   Includes:
   - Copper losses (I²R)
   - Iron losses
   - Friction losses
   - Gearbox losses
"""

import yaml
from typing import Dict, Tuple
import numpy as np


class PowerCalculator:
    """Calculate power and current consumption for robot actuators."""

    def __init__(self, actuator_specs_path: str):
        """
        Initialize power calculator with actuator specifications.

        Args:
            actuator_specs_path: Path to actuator_specs.yaml file
        """
        with open(actuator_specs_path, 'r') as f:
            self.specs = yaml.safe_load(f)

        self.actuators = self.specs['actuators']

        # Joint name to actuator mapping
        self.joint_to_actuator = {
            'left_shoulder_pitch_rs04': 'left_shoulder_pitch_rs04',
            'left_shoulder_roll_rs04': 'left_shoulder_roll_rs04',
            'left_shoulder_yaw_rs03': 'left_shoulder_yaw_rs03',
            'left_elbow_rs03': 'left_elbow_rs03',
            'left_wrist_rs02': 'left_wrist_rs02',
            'left_hand_rs02': 'left_wrist_rs02',  # Same motor type as wrist
        }

        # Brake states (default: all released)
        self.brake_states = {name: False for name in self.joint_to_actuator.keys()}

    def calculate_motor_current(
        self,
        joint_name: str,
        torque: float,
        velocity: float,
        include_friction: bool = True
    ) -> float:
        """
        Calculate motor current consumption.

        Args:
            joint_name: Name of the joint
            torque: Output torque at joint shaft (Nm)
            velocity: Joint velocity (rad/s)
            include_friction: Include friction compensation current

        Returns:
            Current in Amperes
        """
        actuator_name = self.joint_to_actuator.get(joint_name)
        if not actuator_name:
            return 0.0

        motor = self.actuators[actuator_name]['motor']

        # Motor parameters
        kt = motor['motor_constant_kt']  # Nm/A
        gr = motor['gear_ratio']
        eta = motor['gear_efficiency']

        # Torque-related current (main component)
        # I_torque = τ / (Kt * η * GR)
        # Note: For output torque, we need to account for gearbox
        i_torque = abs(torque) / (kt * eta * gr) if kt > 0 else 0.0

        # No-load current (estimated from friction)
        i_no_load = 0.0
        if include_friction and abs(velocity) > 0.01:
            # Friction torque at output
            friction_viscous = motor.get('friction_viscous', 0.0)
            friction_coulomb = motor.get('friction_coulomb', 0.0)

            tau_friction = friction_viscous * abs(velocity) + friction_coulomb
            i_no_load = tau_friction / (kt * eta * gr) if kt > 0 else 0.0

        # Total current
        current = i_torque + i_no_load

        # Clamp to peak current limit
        peak_current = motor.get('peak_current', 100.0)
        current = min(current, peak_current)

        return current

    def calculate_motor_voltage(
        self,
        joint_name: str,
        current: float,
        velocity: float
    ) -> float:
        """
        Calculate motor supply voltage.

        Args:
            joint_name: Name of the joint
            current: Motor current (A)
            velocity: Joint velocity (rad/s)

        Returns:
            Voltage in Volts
        """
        actuator_name = self.joint_to_actuator.get(joint_name)
        if not actuator_name:
            return 0.0

        motor = self.actuators[actuator_name]['motor']

        # Motor parameters
        ke = motor['motor_constant_ke']  # V·s/rad
        r = motor['motor_resistance']  # Ohm
        gr = motor['gear_ratio']

        # Motor velocity (before gearbox)
        omega_motor = abs(velocity) * gr

        # Back-EMF voltage
        v_bemf = ke * omega_motor

        # Resistive voltage drop
        v_resistive = current * r

        # Total voltage
        voltage = v_bemf + v_resistive

        # Clamp to rated voltage
        rated_voltage = motor.get('rated_voltage', 48.0)
        voltage = min(voltage, rated_voltage)

        return voltage

    def calculate_electrical_power(
        self,
        joint_name: str,
        torque: float,
        velocity: float,
        use_supply_voltage: bool = True
    ) -> Tuple[float, float, float]:
        """
        Calculate electrical power consumption.

        Args:
            joint_name: Name of the joint
            torque: Output torque (Nm)
            velocity: Joint velocity (rad/s)
            use_supply_voltage: If True, use rated supply voltage (48V) for power calc.
                              If False, use calculated motor terminal voltage.

        Returns:
            Tuple of (current_A, voltage_V, power_W)
        """
        actuator_name = self.joint_to_actuator.get(joint_name)
        current = self.calculate_motor_current(joint_name, torque, velocity)

        if use_supply_voltage and actuator_name:
            # Use supply voltage for power consumption from battery
            motor = self.actuators[actuator_name]['motor']
            voltage = motor.get('rated_voltage', 48.0)
        else:
            # Use calculated motor terminal voltage (back-EMF + I*R)
            voltage = self.calculate_motor_voltage(joint_name, current, velocity)

        power = voltage * current

        return current, voltage, power

    def calculate_mechanical_power(
        self,
        torque: float,
        velocity: float
    ) -> float:
        """
        Calculate mechanical power output.

        Args:
            torque: Output torque (Nm)
            velocity: Joint velocity (rad/s)

        Returns:
            Mechanical power in Watts
        """
        return abs(torque * velocity)

    def calculate_efficiency(
        self,
        joint_name: str,
        torque: float,
        velocity: float
    ) -> float:
        """
        Calculate instantaneous efficiency.

        Args:
            joint_name: Name of the joint
            torque: Output torque (Nm)
            velocity: Joint velocity (rad/s)

        Returns:
            Efficiency (0.0 to 1.0)
        """
        p_mech = self.calculate_mechanical_power(torque, velocity)
        _, _, p_elec = self.calculate_electrical_power(joint_name, torque, velocity)

        if p_elec < 1e-6:  # Avoid division by zero
            return 0.0

        return min(p_mech / p_elec, 1.0)

    def set_brake_state(self, joint_name: str, engaged: bool):
        """
        Set brake engagement state.

        Args:
            joint_name: Name of the joint
            engaged: True if brake is engaged (holding), False if released
        """
        if joint_name in self.brake_states:
            self.brake_states[joint_name] = engaged

    def get_brake_power(self, joint_name: str) -> float:
        """
        Get brake power consumption.

        Args:
            joint_name: Name of the joint

        Returns:
            Brake power in Watts (0 if released)
        """
        if not self.brake_states.get(joint_name, False):
            return 0.0

        actuator_name = self.joint_to_actuator.get(joint_name)
        if not actuator_name:
            return 0.0

        motor = self.actuators[actuator_name]['motor']
        return motor.get('brake_power', 0.0)

    def calculate_total_power(
        self,
        joint_states: Dict[str, Dict[str, float]]
    ) -> Dict[str, any]:
        """
        Calculate total system power consumption.

        Args:
            joint_states: Dictionary mapping joint names to states
                         Each state is a dict with 'position', 'velocity', 'effort' (torque)

        Returns:
            Dictionary with power statistics:
            {
                'joints': {joint_name: {'current': A, 'voltage': V, 'power': W}},
                'total_current': A,
                'total_power': W,
                'motor_power': W,
                'brake_power': W,
                'mechanical_power': W,
                'efficiency': ratio
            }
        """
        result = {
            'joints': {},
            'total_current': 0.0,
            'total_power': 0.0,
            'motor_power': 0.0,
            'brake_power': 0.0,
            'mechanical_power': 0.0,
            'efficiency': 0.0
        }

        for joint_name, state in joint_states.items():
            if joint_name not in self.joint_to_actuator:
                continue

            torque = state.get('effort', 0.0)
            velocity = state.get('velocity', 0.0)

            # Calculate electrical power
            current, voltage, power = self.calculate_electrical_power(
                joint_name, torque, velocity
            )

            # Calculate mechanical power
            p_mech = self.calculate_mechanical_power(torque, velocity)

            # Get brake power
            p_brake = self.get_brake_power(joint_name)

            # Store per-joint data
            result['joints'][joint_name] = {
                'current': current,
                'voltage': voltage,
                'power': power,
                'mechanical_power': p_mech,
                'brake_power': p_brake,
                'efficiency': self.calculate_efficiency(joint_name, torque, velocity)
            }

            # Accumulate totals
            result['total_current'] += current
            result['motor_power'] += power
            result['brake_power'] += p_brake
            result['mechanical_power'] += p_mech

        # Total power includes motors and brakes
        result['total_power'] = result['motor_power'] + result['brake_power']

        # Overall system efficiency
        if result['motor_power'] > 1e-6:
            result['efficiency'] = result['mechanical_power'] / result['motor_power']
        else:
            result['efficiency'] = 0.0

        return result

    def estimate_battery_life(
        self,
        average_power_w: float,
        battery_capacity_ah: float,
        battery_voltage_v: float = 48.0,
        depth_of_discharge: float = 0.8
    ) -> float:
        """
        Estimate battery life.

        Args:
            average_power_w: Average power consumption (W)
            battery_capacity_ah: Battery capacity (Ah)
            battery_voltage_v: Battery nominal voltage (V)
            depth_of_discharge: Maximum DOD (0.8 = 80%)

        Returns:
            Estimated runtime in hours
        """
        if average_power_w < 1e-6:
            return float('inf')

        # Available energy
        energy_wh = battery_capacity_ah * battery_voltage_v * depth_of_discharge

        # Runtime
        runtime_h = energy_wh / average_power_w

        return runtime_h


def main():
    """Example usage."""
    import os
    from ament_index_python.packages import get_package_share_directory

    # Load actuator specs
    pkg_share = get_package_share_directory('arm_control')
    specs_path = os.path.join(pkg_share, 'config', 'actuator_specs.yaml')

    calc = PowerCalculator(specs_path)

    # Example: Calculate power for a single joint
    joint_name = 'left_shoulder_pitch_rs04'
    torque = 30.0  # Nm
    velocity = 1.0  # rad/s

    current, voltage, power = calc.calculate_electrical_power(joint_name, torque, velocity, use_supply_voltage=True)

    print(f"\nSingle Joint Power Calculation:")
    print(f"Joint: {joint_name}")
    print(f"Torque: {torque} Nm")
    print(f"Velocity: {velocity} rad/s")
    print(f"Current: {current:.2f} A")
    print(f"Supply Voltage: {voltage:.2f} V (rated supply)")
    print(f"Power Consumption: {power:.2f} W")

    p_mech = calc.calculate_mechanical_power(torque, velocity)
    eff = calc.calculate_efficiency(joint_name, torque, velocity)
    print(f"Mechanical Power: {p_mech:.2f} W")
    print(f"Efficiency: {eff*100:.1f}%")

    # Also show motor terminal voltage
    _, v_motor, _ = calc.calculate_electrical_power(joint_name, torque, velocity, use_supply_voltage=False)
    print(f"Motor Terminal Voltage: {v_motor:.2f} V (back-EMF + I×R)")

    # Example: Calculate total system power
    print("\n" + "="*60)
    print("Total System Power Calculation:")
    print("="*60)

    joint_states = {
        'left_shoulder_pitch_rs04': {'effort': 40.0, 'velocity': 0.5},
        'left_shoulder_roll_rs04': {'effort': 35.0, 'velocity': 0.3},
        'left_shoulder_yaw_rs03': {'effort': 15.0, 'velocity': 0.8},
        'left_elbow_rs03': {'effort': 20.0, 'velocity': 1.0},
        'left_wrist_rs02': {'effort': 5.0, 'velocity': 1.5},
        'left_hand_rs02': {'effort': 3.0, 'velocity': 0.2},
    }

    result = calc.calculate_total_power(joint_states)

    print(f"\nPer-Joint Power Consumption:")
    for joint, data in result['joints'].items():
        print(f"\n{joint}:")
        print(f"  Current:    {data['current']:.2f} A")
        print(f"  Voltage:    {data['voltage']:.2f} V (supply)")
        print(f"  Power:      {data['power']:.2f} W")
        print(f"  Efficiency: {data['efficiency']*100:.1f}%")

    print(f"\n{'='*60}")
    print(f"TOTAL SYSTEM:")
    print(f"  Total Current:      {result['total_current']:.2f} A")
    print(f"  Motor Power:        {result['motor_power']:.2f} W")
    print(f"  Brake Power:        {result['brake_power']:.2f} W")
    print(f"  Total Power:        {result['total_power']:.2f} W")
    print(f"  Mechanical Power:   {result['mechanical_power']:.2f} W")
    print(f"  System Efficiency:  {result['efficiency']*100:.1f}%")

    # Battery life estimation
    battery_ah = 20.0  # 20 Ah battery
    runtime = calc.estimate_battery_life(result['total_power'], battery_ah)
    print(f"\nEstimated Battery Life (20Ah @ 48V):")
    print(f"  Runtime: {runtime:.2f} hours ({runtime*60:.0f} minutes)")


if __name__ == '__main__':
    main()
