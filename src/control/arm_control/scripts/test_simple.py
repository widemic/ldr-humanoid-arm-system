#!/usr/bin/env python3
"""Quick test to verify the simple controller works."""

import rclpy
from motion_planner import MotionPlanner
import time


def main():
    """Run simple test."""
    rclpy.init()

    print("Creating motion planner...")
    planner = MotionPlanner()

    # Wait for initialization
    time.sleep(2.0)

    try:
        # Test 1: Get current position
        print("\n=== Test 1: Get Position ===")
        pos = planner.get_position()
        if pos:
            print(f"✓ Current position: {[f'{p:.2f}' for p in pos]}")
        else:
            print("✗ Failed to get position")
            return

        # Test 2: Move to ready
        print("\n=== Test 2: Move to Ready Pose ===")
        if planner.ready():
            print("✓ Successfully moved to ready pose")
        else:
            print("✗ Failed to move to ready")
            return

        time.sleep(1.0)

        # Test 3: Move home
        print("\n=== Test 3: Move Home ===")
        if planner.home():
            print("✓ Successfully moved home")
        else:
            print("✗ Failed to move home")
            return

        print("\n=== All tests passed! ===")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
