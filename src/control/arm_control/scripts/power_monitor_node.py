#!/usr/bin/env python3
"""
ROS 2 node for real-time power and current consumption monitoring.

Subscribes to:
  /joint_states - Joint positions, velocities, and efforts

Publishes:
  /power_monitor/total_power (std_msgs/Float64) - Total system power in Watts
  /power_monitor/total_current (std_msgs/Float64) - Total current in Amperes
  /power_monitor/detailed (custom msg) - Detailed per-joint power data

Parameters:
  update_rate (float): Publishing rate in Hz (default: 10.0)
  actuator_specs_path (str): Path to actuator_specs.yaml
  log_to_file (bool): Enable CSV logging (default: False)
  log_file_path (str): Path to CSV log file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
import json
import os
import csv
from datetime import datetime

from power_calculator import PowerCalculator


class PowerMonitorNode(Node):
    """ROS 2 node for monitoring actuator power consumption."""

    def __init__(self):
        super().__init__('power_monitor_node')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('actuator_specs_path', '')
        self.declare_parameter('log_to_file', False)
        self.declare_parameter('log_file_path', '/tmp/power_log.csv')

        # Get parameters
        update_rate = self.get_parameter('update_rate').value
        specs_path = self.get_parameter('actuator_specs_path').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_file_path = self.get_parameter('log_file_path').value

        # Load actuator specs
        if not specs_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('arm_control')
                specs_path = os.path.join(pkg_share, 'config', 'actuator_specs.yaml')
            except:
                self.get_logger().error('Cannot find actuator_specs.yaml!')
                raise

        self.get_logger().info(f'Loading actuator specs from: {specs_path}')
        self.power_calc = PowerCalculator(specs_path)

        # Initialize state storage
        self.latest_joint_states = {}
        self.last_update_time = self.get_clock().now()

        # Statistics
        self.total_energy_wh = 0.0  # Cumulative energy consumption
        self.peak_power_w = 0.0
        self.avg_power_w = 0.0
        self.sample_count = 0

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.total_power_pub = self.create_publisher(Float64, '/power_monitor/total_power', 10)
        self.total_current_pub = self.create_publisher(Float64, '/power_monitor/total_current', 10)
        self.detailed_pub = self.create_publisher(String, '/power_monitor/detailed', 10)
        self.statistics_pub = self.create_publisher(String, '/power_monitor/statistics', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / update_rate, self.publish_power_data)

        # CSV logging
        if self.log_to_file:
            self.setup_csv_logging()

        self.get_logger().info(f'Power Monitor Node started (update rate: {update_rate} Hz)')
        if self.log_to_file:
            self.get_logger().info(f'Logging to: {self.log_file_path}')

    def setup_csv_logging(self):
        """Initialize CSV logging."""
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        header = ['timestamp', 'total_power_W', 'total_current_A', 'motor_power_W',
                  'brake_power_W', 'mechanical_power_W', 'efficiency']

        # Add per-joint columns
        for joint in self.power_calc.joint_to_actuator.keys():
            header.extend([
                f'{joint}_current_A',
                f'{joint}_voltage_V',
                f'{joint}_power_W'
            ])

        self.csv_writer.writerow(header)
        self.csv_file.flush()

    def joint_state_callback(self, msg: JointState):
        """Store latest joint states."""
        for i, name in enumerate(msg.name):
            self.latest_joint_states[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0
            }

    def publish_power_data(self):
        """Calculate and publish power consumption data."""
        if not self.latest_joint_states:
            # Log warning every 20 calls (every 2 seconds at 10Hz)
            if self.sample_count % 20 == 0:
                self.get_logger().warn('No joint states received. Is the simulation running?')
            self.sample_count += 1
            return

        # Calculate power
        result = self.power_calc.calculate_total_power(self.latest_joint_states)

        # Update statistics
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9  # seconds
        self.last_update_time = current_time

        # Energy integration (Wh) - avoid NaN
        if result['total_power'] >= 0 and not (result['total_power'] != result['total_power']):  # Check for NaN
            self.total_energy_wh += result['total_power'] * dt / 3600.0

        # Peak power
        if result['total_power'] > self.peak_power_w:
            self.peak_power_w = result['total_power']

        # Running average
        self.sample_count += 1
        alpha = 1.0 / min(self.sample_count, 100)  # Exponential moving average
        if result['total_power'] >= 0 and not (result['total_power'] != result['total_power']):  # Check for NaN
            self.avg_power_w = (1 - alpha) * self.avg_power_w + alpha * result['total_power']

        # Publish total power
        total_power_msg = Float64()
        total_power_msg.data = result['total_power']
        self.total_power_pub.publish(total_power_msg)

        # Publish total current
        total_current_msg = Float64()
        total_current_msg.data = result['total_current']
        self.total_current_pub.publish(total_current_msg)

        # Publish detailed data (JSON)
        detailed_msg = String()
        detailed_msg.data = json.dumps(result['joints'], indent=2)
        self.detailed_pub.publish(detailed_msg)

        # Publish statistics
        stats = {
            'total_power_W': result['total_power'],
            'total_current_A': result['total_current'],
            'motor_power_W': result['motor_power'],
            'brake_power_W': result['brake_power'],
            'mechanical_power_W': result['mechanical_power'],
            'efficiency': result['efficiency'],
            'cumulative_energy_Wh': self.total_energy_wh,
            'peak_power_W': self.peak_power_w,
            'average_power_W': self.avg_power_w
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats, indent=2)
        self.statistics_pub.publish(stats_msg)

        # Log to CSV
        if self.log_to_file:
            self.log_to_csv(result)

        # Log to console (every 2 seconds)
        if self.sample_count % 20 == 0:
            self.get_logger().info(
                f'Power: {result["total_power"]:.1f}W | '
                f'Current: {result["total_current"]:.1f}A | '
                f'Efficiency: {result["efficiency"]*100:.1f}% | '
                f'Energy: {self.total_energy_wh:.3f}Wh'
            )

    def log_to_csv(self, result):
        """Log power data to CSV file."""
        timestamp = datetime.now().isoformat()
        row = [
            timestamp,
            result['total_power'],
            result['total_current'],
            result['motor_power'],
            result['brake_power'],
            result['mechanical_power'],
            result['efficiency']
        ]

        # Add per-joint data
        for joint in self.power_calc.joint_to_actuator.keys():
            if joint in result['joints']:
                data = result['joints'][joint]
                row.extend([
                    data['current'],
                    data['voltage'],
                    data['power']
                ])
            else:
                row.extend([0.0, 0.0, 0.0])

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def __del__(self):
        """Cleanup."""
        if self.log_to_file and hasattr(self, 'csv_file'):
            self.csv_file.close()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = PowerMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
