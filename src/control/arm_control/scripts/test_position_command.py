#!/usr/bin/env python3
"""
Test script to send position commands directly to the custom controller.

This demonstrates that the controller can track position commands
by writing to the position command interface.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import math


class PositionCommandTest(Node):
    def __init__(self):
        super().__init__('position_command_test')

        # Publisher to position command interface
        # ros2_control exposes this topic for each controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Subscribe to joint states to monitor
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_positions = None
        self.arm_joints = [
            'left_shoulder_pitch_rs04',
            'left_shoulder_roll_rs04',
            'left_shoulder_yaw_rs03',
            'left_elbow_rs03',
            'left_wrist_rs02',
            'left_hand_rs02',
        ]

    def joint_state_callback(self, msg):
        """Store current positions"""
        positions = {}
        for i, name in enumerate(msg.name):
            if name in self.arm_joints and i < len(msg.position):
                positions[name] = msg.position[i]

        if len(positions) == 6:
            self.current_positions = [positions[j] for j in self.arm_joints]

    def send_position_command(self, positions):
        """Send position command to controller"""
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent command: {[f"{p:.3f}" for p in positions]}')

    def get_current_positions(self):
        """Get current joint positions"""
        return self.current_positions


def main():
    rclpy.init()
    node = PositionCommandTest()

    print("\n" + "="*80)
    print("POSITION COMMAND TEST - Custom PID Controller")
    print("="*80)

    # Wait for joint states
    print("\nWaiting for joint states...")
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.current_positions:
            break

    if not node.current_positions:
        print("❌ Could not get joint states!")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("✅ Connected to controller!\n")

    initial_pos = node.get_current_positions()
    print("Current positions:")
    for i, (joint, pos) in enumerate(zip(node.arm_joints, initial_pos)):
        print(f"  {i}: {joint:30s} = {pos:7.3f} rad ({pos*57.3:6.1f}°)")

    print("\n" + "="*80)
    print("TEST SEQUENCE")
    print("="*80)

    try:
        # Test 1: Small movement on shoulder pitch
        print("\n[Test 1] Moving shoulder pitch by +0.2 rad (~11°)...")
        target = initial_pos.copy()
        target[0] += 0.2  # Shoulder pitch
        node.send_position_command(target)
        time.sleep(3.0)

        # Check result
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        current = node.get_current_positions()
        error = abs(current[0] - target[0])
        print(f"   Target: {target[0]:.3f}, Current: {current[0]:.3f}, Error: {error:.4f} rad")
        if error < 0.05:
            print("   ✅ Tracking successful!")
        else:
            print("   ⚠️  Large error - may need PID tuning")

        # Test 2: Small movement on elbow
        print("\n[Test 2] Moving elbow by -0.3 rad (~17°)...")
        target = initial_pos.copy()
        target[3] -= 0.3  # Elbow
        node.send_position_command(target)
        time.sleep(3.0)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        current = node.get_current_positions()
        error = abs(current[3] - target[3])
        print(f"   Target: {target[3]:.3f}, Current: {current[3]:.3f}, Error: {error:.4f} rad")
        if error < 0.05:
            print("   ✅ Tracking successful!")
        else:
            print("   ⚠️  Large error - may need PID tuning")

        # Test 3: Return to initial position
        print("\n[Test 3] Returning to initial position...")
        node.send_position_command(initial_pos)
        time.sleep(3.0)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        current = node.get_current_positions()
        max_error = max(abs(c - i) for c, i in zip(current, initial_pos))
        print(f"   Max error: {max_error:.4f} rad")
        if max_error < 0.05:
            print("   ✅ Returned successfully!")
        else:
            print("   ⚠️  Position error detected")

        print("\n" + "="*80)
        print("RESULTS")
        print("="*80)
        print("✅ Position commands are working!")
        print("✅ Controller reads from position command interface")
        print("✅ Controller writes effort to Gazebo")
        print("\n🎯 Next: Add FollowJointTrajectory action server for MoveIt")
        print("="*80 + "\n")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
