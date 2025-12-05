#!/usr/bin/env python3
"""
Single Joint PID Tuner

This script helps you tune PID gains for one joint at a time by:
1. Commanding the selected joint to oscillate between two positions
2. Keeping all other joints stationary
3. Providing real-time feedback on tracking performance

While this runs, you can dynamically adjust PID gains using:
  ros2 param set /arm_controller pid.kp "[100, 100, 80, NEW_VALUE, 60, 60]"
  ros2 param set /arm_controller pid.kd "[2.0, 2.0, 1.5, NEW_VALUE, 1.0, 1.0]"

Usage:
  ros2 run arm_control tune_single_joint.py --joint 3  # Tune elbow
  ros2 run arm_control tune_single_joint.py --joint 0  # Tune shoulder pitch
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import argparse
import sys
import math

class SingleJointTuner(Node):
    def __init__(self, joint_index, oscillation_period=3.0, amplitude=0.5, max_velocity=1.0, direction=1):
        super().__init__('single_joint_tuner')

        # Joint names for reference
        self.joint_names = [
            'left_shoulder_pitch_rs04',  # 0
            'left_shoulder_roll_rs04',   # 1
            'left_shoulder_yaw_rs03',    # 2
            'left_elbow_rs03',           # 3
            'left_wrist_rs02',           # 4
            'left_hand_rs02'             # 5
        ]

        # Validate joint index
        if joint_index < 0 or joint_index >= len(self.joint_names):
            self.get_logger().error(f'Invalid joint index {joint_index}. Must be 0-5.')
            sys.exit(1)

        self.joint_index = joint_index
        self.joint_name = self.joint_names[joint_index]
        self.oscillation_period = oscillation_period
        self.amplitude = amplitude
        self.max_velocity = max_velocity
        self.direction = 1 if direction >= 0 else -1

        # Publisher for commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Subscriber for joint states (to monitor performance)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current state tracking
        self.current_position = None
        self.current_velocity = None
        self.current_effort = None
        self.desired_position = 0.0

        # Test motion: oscillate between two positions
        self.position_a = 0.0
        self.position_b = amplitude * self.direction
        self.toggle = False

        # For smooth motion
        self.target_position = 0.0
        self.command_position = 0.0
        self.last_update_time = self.get_clock().now()

        # Performance metrics
        self.max_error = 0.0
        self.max_effort = 0.0

        # Print header
        self.print_header()

        # Create timer for target switching
        self.timer = self.create_timer(self.oscillation_period, self.toggle_target)

        # Create timer for smooth command updates (100 Hz)
        self.command_timer = self.create_timer(0.01, self.update_command)

        # Create timer for status updates
        self.status_timer = self.create_timer(0.1, self.print_status)

    def toggle_target(self):
        """Switch target position"""
        if self.toggle:
            self.target_position = self.position_b
            direction = "→"
        else:
            self.target_position = self.position_a
            direction = "←"

        self.get_logger().info(
            f'{direction} Target: {self.target_position:+.3f} rad '
            f'({math.degrees(self.target_position):+.1f}°) | '
            f'Max error: {self.max_error:.4f} rad | '
            f'Max effort: {self.max_effort:.1f} Nm'
        )

        # Reset max trackers for next cycle
        self.max_error = 0.0
        self.max_effort = 0.0

        self.toggle = not self.toggle

    def update_command(self):
        """Update command with velocity limiting (100 Hz)"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        if dt <= 0:
            return

        # Calculate position error to target
        error = self.target_position - self.command_position

        # Limit velocity
        max_step = self.max_velocity * dt
        if abs(error) > max_step:
            step = max_step if error > 0 else -max_step
        else:
            step = error

        self.command_position += step
        self.desired_position = self.command_position

        # Send command
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.data[self.joint_index] = self.command_position
        self.publisher.publish(msg)

    def print_header(self):
        print("\n" + "="*70)
        print("SINGLE JOINT PID TUNER")
        print("="*70)
        print(f"\nTuning joint {self.joint_index}: {self.joint_name}")
        print(f"Oscillation period: {self.oscillation_period}s")
        print(f"Amplitude: ±{self.amplitude} rad (±{math.degrees(self.amplitude):.1f}°)")
        print(f"Max velocity: {self.max_velocity} rad/s ({math.degrees(self.max_velocity):.1f}°/s)")
        print("\nWhile this runs, adjust PID gains using:")
        print(f"  ros2 param set /arm_controller pid.kp \"[100, 100, 80, NEW_KP, 60, 60]\"")
        print(f"  ros2 param set /arm_controller pid.kd \"[2.0, 2.0, 1.5, NEW_KD, 1.0, 1.0]\"")
        print(f"                                              ^^^")
        print(f"                                         Joint index {self.joint_index}")
        print("\nWhat to watch for:")
        print("  • OVERSHOOT      → Reduce Kp or increase Kd")
        print("  • OSCILLATION    → Reduce Kd")
        print("  • SLOW RESPONSE  → Increase Kp")
        print("  • STEADY ERROR   → Increase Ki (use cautiously!)")
        print("  • HIGH EFFORT    → Gains too aggressive")
        print("\nPress Ctrl+C to stop")
        print("="*70)
        print(f"{'Time':<8} {'Desired':<10} {'Actual':<10} {'Error':<10} {'Velocity':<10} {'Effort':<10}")
        print("-"*70)

    def joint_state_callback(self, msg):
        """Monitor the joint being tuned"""
        try:
            # Find the joint we're tuning in the joint_states message
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            self.current_velocity = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
            self.current_effort = msg.effort[idx] if len(msg.effort) > idx else 0.0

            # Track max values
            error = abs(self.desired_position - self.current_position)
            self.max_error = max(self.max_error, error)
            self.max_effort = max(self.max_effort, abs(self.current_effort))

        except (ValueError, IndexError):
            # Joint not found in message yet
            pass

    def print_status(self):
        """Print current joint state (10 Hz)"""
        if self.current_position is not None:
            error = self.desired_position - self.current_position

            # Print compact status line
            print(
                f"{self.get_clock().now().nanoseconds/1e9:8.1f} "
                f"{self.desired_position:+10.4f} "
                f"{self.current_position:+10.4f} "
                f"{error:+10.4f} "
                f"{self.current_velocity:+10.4f} "
                f"{self.current_effort:+10.2f}",
                end='\r'  # Overwrite same line
            )

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Tune PID gains for a single joint')
    parser.add_argument(
        '--joint', '-j',
        type=int,
        default=3,
        help='Joint index to tune (0=shoulder_pitch, 1=shoulder_roll, 2=shoulder_yaw, '
             '3=elbow, 4=wrist_pitch, 5=wrist_roll). Default: 3 (elbow)'
    )
    parser.add_argument(
        '--period', '-p',
        type=float,
        default=3.0,
        help='Oscillation period in seconds. Default: 3.0'
    )
    parser.add_argument(
        '--amplitude', '-a',
        type=float,
        default=0.5,
        help='Oscillation amplitude in radians. Default: 0.5 (~28°)'
    )
    parser.add_argument(
        '--velocity', '-v',
        type=float,
        default=1.0,
        help='Maximum velocity in radians/second. Default: 1.0'
    )
    parser.add_argument(
        '--direction', '-d',
        type=int,
        default=1,
        help='Direction of oscillation: 1 for positive, -1 for negative. Default: 1'
    )

    args = parser.parse_args()

    # Initialize ROS 2
    rclpy.init()

    try:
        node = SingleJointTuner(
            joint_index=args.joint,
            oscillation_period=args.period,
            amplitude=args.amplitude,
            max_velocity=args.velocity,
            direction=args.direction
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Tuning stopped by user")
        print("="*70)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
