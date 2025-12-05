#!/usr/bin/env python3
"""
Verify that PID gains are actually being used by the controller.

This script:
1. Monitors controller startup logs for loaded PID gains
2. Sends a command to move the elbow joint
3. Monitors effort output and behavior
4. Shows if high Kp=4000 is actually affecting control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import sys


class PIDVerifier(Node):
    def __init__(self):
        super().__init__('pid_verifier')

        # Publisher for commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Subscriber for joint states
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_state = None
        self.effort_samples = []
        self.arm_joints = [
            'left_shoulder_pitch_rs04',
            'left_shoulder_roll_rs04',
            'left_shoulder_yaw_rs03',
            'left_elbow_rs03',
            'left_wrist_rs02',
            'left_hand_rs02',
        ]

    def joint_state_callback(self, msg):
        """Store joint states"""
        state = {}
        for i, name in enumerate(msg.name):
            if name in self.arm_joints:
                state[name] = {
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
                }

        if len(state) == 6:
            self.current_state = state

            # Collect effort samples for elbow
            if 'left_elbow_rs03' in state:
                self.effort_samples.append(abs(state['left_elbow_rs03']['effort']))

    def get_joint_positions(self):
        """Get current joint positions as list"""
        if not self.current_state:
            return None
        return [self.current_state[j]['position'] for j in self.arm_joints]

    def send_command(self, positions):
        """Send position command"""
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PIDVerifier()

    print("\n" + "="*80)
    print("PID VERIFICATION TEST")
    print("="*80)
    print("\nThis test verifies that PID gains are actually being used.")
    print("Expected behavior with Kp=4000 on elbow:")
    print("  - Very high effort values (hundreds of Nm)")
    print("  - Fast/aggressive response")
    print("  - Possible oscillation/overshoot")
    print("\n" + "="*80)

    # Wait for joint states
    print("\nWaiting for joint states...")
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.current_state:
            break

    if not node.current_state:
        print("❌ Could not get joint states! Is the simulation running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("✅ Connected!\n")

    # Get initial position
    initial_pos = node.get_joint_positions()
    print("Initial positions:")
    for i, (joint, pos) in enumerate(zip(node.arm_joints, initial_pos)):
        effort = node.current_state[joint]['effort']
        print(f"  {i}: {joint:30s} = {pos:7.3f} rad, effort = {effort:7.2f} Nm")

    print("\n" + "="*80)
    print("CHECK CONTROLLER LOGS")
    print("="*80)
    print("Look at the terminal where the system was launched.")
    print("You should see controller startup logs showing:")
    print("  Joint 3 (left_elbow_rs03): Kp=4000.00, ...")
    print("  ⚠️  VERY HIGH Kp DETECTED: 4000.00 (testing PID functionality)")
    print("\nIf you see these messages, the config file is being loaded correctly.")

    input("\nPress ENTER when you've checked the logs...")

    print("\n" + "="*80)
    print("TEST: Move elbow joint")
    print("="*80)

    # Clear effort samples
    node.effort_samples = []

    # Send command to move elbow by 0.3 rad
    target = initial_pos.copy()
    target[3] += 0.3  # Elbow

    print(f"\nSending command to move elbow from {initial_pos[3]:.3f} to {target[3]:.3f} rad")
    print("Monitoring effort values for 3 seconds...\n")

    node.send_command(target)

    # Monitor for 3 seconds
    start_time = time.time()
    max_effort = 0.0

    while time.time() - start_time < 3.0:
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.current_state:
            elbow_effort = abs(node.current_state['left_elbow_rs03']['effort'])
            max_effort = max(max_effort, elbow_effort)

            # Print periodic updates
            elapsed = time.time() - start_time
            if int(elapsed * 2) % 1 == 0:  # Every 0.5s
                elbow_pos = node.current_state['left_elbow_rs03']['position']
                error = target[3] - elbow_pos
                print(f"  t={elapsed:.1f}s: pos={elbow_pos:.3f}, error={error:.4f}, effort={elbow_effort:.1f} Nm")

    # Analyze results
    print("\n" + "="*80)
    print("RESULTS")
    print("="*80)

    final_pos = node.current_state['left_elbow_rs03']['position']
    final_error = abs(target[3] - final_pos)
    avg_effort = sum(node.effort_samples) / len(node.effort_samples) if node.effort_samples else 0

    print(f"\nTarget position:     {target[3]:.3f} rad")
    print(f"Final position:      {final_pos:.3f} rad")
    print(f"Final error:         {final_error:.4f} rad")
    print(f"Max effort observed: {max_effort:.1f} Nm")
    print(f"Avg effort:          {avg_effort:.1f} Nm")

    print("\n" + "="*80)
    print("ANALYSIS")
    print("="*80)

    if max_effort > 100:
        print("✅ HIGH EFFORT DETECTED!")
        print(f"   Max effort {max_effort:.1f} Nm indicates Kp=4000 is being used.")
        print("   The PID controller is working correctly.")
    elif max_effort > 20:
        print("⚠️  MODERATE EFFORT")
        print(f"   Max effort {max_effort:.1f} Nm is higher than expected for Kp=40,")
        print("   but lower than expected for Kp=4000.")
        print("   This suggests the gains might not be fully applied.")
    else:
        print("❌ LOW EFFORT")
        print(f"   Max effort {max_effort:.1f} Nm is too low for Kp=4000.")
        print("   This indicates the PID gains are NOT being used.")
        print("   Possible issues:")
        print("   - Config file not being loaded")
        print("   - Controller not using PID computation")
        print("   - Effort limiting somewhere in the pipeline")

    if final_error < 0.01:
        print(f"\n✅ Good tracking: final error {final_error:.4f} rad")
    else:
        print(f"\n⚠️  Poor tracking: final error {final_error:.4f} rad")

    print("\n" + "="*80)
    print("RECOMMENDATION")
    print("="*80)

    if max_effort > 100:
        print("The PID is working! Now change Kp back to a reasonable value:")
        print("  Edit: src/control/arm_control/config/controllers.yaml")
        print("  Change: kp: [50.0, 50.0, 40.0, 4000.0, 30.0, 30.0]")
        print("  To:     kp: [50.0, 50.0, 40.0, 40.0, 30.0, 30.0]")
        print("  Then rebuild and restart the system.")
    else:
        print("Something is wrong. The high Kp=4000 should produce much higher effort.")
        print("Check the controller startup logs for the loaded PID values.")

    print("="*80 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
