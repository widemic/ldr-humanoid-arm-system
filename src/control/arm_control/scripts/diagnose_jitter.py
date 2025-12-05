#!/usr/bin/env python3
"""
Quick diagnostic tool to check for controller jitter/oscillation.
Monitors joint states and reports statistics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from collections import deque


class JitterDiagnostic(Node):
    def __init__(self):
        super().__init__('jitter_diagnostic')

        self.joint_names = [
            'left_shoulder_pitch_rs04',
            'left_shoulder_roll_rs04',
            'left_shoulder_yaw_rs03',
            'left_elbow_rs03',
            'left_wrist_rs02',
            'left_hand_rs02',
        ]

        # Store recent values (last 100 samples)
        self.position_history = {name: deque(maxlen=100) for name in self.joint_names}
        self.velocity_history = {name: deque(maxlen=100) for name in self.joint_names}
        self.effort_history = {name: deque(maxlen=100) for name in self.joint_names}

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Print stats every 2 seconds
        self.timer = self.create_timer(2.0, self.print_stats)

        self.sample_count = 0

    def joint_state_callback(self, msg):
        """Store joint state data"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                if i < len(msg.position):
                    self.position_history[name].append(msg.position[i])
                if i < len(msg.velocity):
                    self.velocity_history[name].append(msg.velocity[i])
                if i < len(msg.effort):
                    self.effort_history[name].append(msg.effort[i])

        self.sample_count += 1

    def print_stats(self):
        """Print statistics about joint behavior"""
        if self.sample_count < 10:
            self.get_logger().info('Collecting data...')
            return

        print("\n" + "="*80)
        print("JOINT STATISTICS (Last 100 samples @ 50Hz = 2 seconds)")
        print("="*80)

        for name in self.joint_names:
            if len(self.position_history[name]) < 10:
                continue

            pos = np.array(self.position_history[name])
            vel = np.array(self.velocity_history[name])
            eff = np.array(self.effort_history[name])

            # Calculate statistics
            pos_std = np.std(pos)
            vel_std = np.std(vel)
            vel_mean = np.mean(np.abs(vel))
            eff_std = np.std(eff)
            eff_mean = np.mean(np.abs(eff))

            # Position variation (detect oscillation)
            pos_range = np.max(pos) - np.min(pos)

            # Detect high-frequency oscillation
            if len(pos) > 5:
                # Check sign changes in velocity (zero crossings)
                vel_sign_changes = np.sum(np.diff(np.sign(vel)) != 0)
                oscillation_freq = vel_sign_changes / 2.0  # Hz (in 2 second window)
            else:
                oscillation_freq = 0

            print(f"\n{name}:")
            print(f"  Position: {pos[-1]:7.4f} rad  (range: {pos_range:7.4f}, std: {pos_std:7.5f})")
            print(f"  Velocity: {vel_mean:7.4f} rad/s avg  (std: {vel_std:7.4f})")
            print(f"  Effort:   {eff_mean:7.2f} Nm avg     (std: {eff_std:7.2f})")
            print(f"  Oscillation: ~{oscillation_freq:.1f} Hz", end="")

            # Diagnosis
            if pos_range > 0.05:  # > 3 degrees
                print("  ⚠️  LARGE MOVEMENT/OSCILLATION")
            elif pos_std > 0.001:  # > 0.06 degrees
                print("  ⚠️  JITTERY")
            elif vel_std > 0.1:
                print("  ⚠️  NOISY VELOCITY")
            else:
                print("  ✅ STABLE")

        print("\n" + "="*80)
        print("DIAGNOSIS:")

        # Overall diagnosis
        max_pos_std = max(np.std(np.array(self.position_history[n]))
                         for n in self.joint_names if len(self.position_history[n]) > 0)

        if max_pos_std > 0.01:
            print("🔴 HIGH JITTER - Gains are still too high!")
            print("   → Reduce Kp and Kd further (try 50% of current values)")
        elif max_pos_std > 0.001:
            print("🟡 MILD JITTER - Acceptable but could be better")
            print("   → Try reducing Kd by 30-50%")
        else:
            print("🟢 STABLE - Controller is well-tuned!")
            print("   → You can carefully increase Kp for better response")

        print("="*80 + "\n")


def main():
    rclpy.init()
    node = JitterDiagnostic()

    print("\n" + "="*80)
    print("JITTER DIAGNOSTIC TOOL")
    print("="*80)
    print("Monitoring /joint_states topic...")
    print("Statistics will update every 2 seconds.")
    print("Press Ctrl+C to stop.")
    print("="*80 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
