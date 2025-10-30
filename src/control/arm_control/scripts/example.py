#!/usr/bin/env python3
"""
Example usage of the motion planner.

Shows how to control the robot arm in Gazebo using simple commands.
"""

import rclpy
from motion_planner import MotionPlanner
import time


def main():
    """Run example motions."""
    rclpy.init()

    # Create planner
    planner = MotionPlanner()

    # Wait for system to initialize
    print("Waiting for robot...")
    time.sleep(2.0)

    try:
        # Example 1: Move to predefined poses
        print("\n=== Example 1: Predefined Poses ===")

        print("Moving to ready position...")
        planner.ready()
        time.sleep(1.0)

        print("Moving home...")
        planner.home()
        time.sleep(1.0)

        # Example 2: Move to specific joint positions
        print("\n=== Example 2: Custom Positions ===")

        print("Moving to custom position...")
        planner.move_to([0.5, 1.0, -0.5, 1.2, 0.3], duration=3.0)
        time.sleep(1.0)

        print("Moving to another position...")
        planner.move_to([-0.5, 0.8, 0.5, 1.5, -0.3], duration=3.0)
        time.sleep(1.0)

        # Example 3: Execute a trajectory with multiple waypoints
        print("\n=== Example 3: Multi-Point Trajectory ===")

        waypoints = [
            ([0.0, 0.5, 0.0, 0.5, 0.0], 2.0),   # First waypoint at t=2s
            ([0.5, 1.0, -0.5, 1.0, 0.5], 4.0),  # Second at t=4s
            ([0.0, 1.5, 0.0, 1.5, 0.0], 6.0),   # Third at t=6s
            ([0.0, 0.0, 0.0, 0.0, 0.0], 8.0),   # Home at t=8s
        ]

        print("Executing smooth trajectory...")
        planner.move_trajectory(waypoints)

        print("\n=== All examples complete! ===")

        # Show final position
        final_pos = planner.get_position()
        if final_pos:
            print(f"Final position: {[f'{p:.3f}' for p in final_pos]}")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
