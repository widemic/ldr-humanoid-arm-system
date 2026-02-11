#!/usr/bin/env python3
"""
MoveIt Task Constructor Simple Demo

A simplified demonstration of MTC showing basic task composition:
- Move to home position
- Approach a target pose
- Retreat back
- Return to home

This example is ideal for learning MTC concepts without gripper complexity.

Author: LDR Robotics Team
License: MIT
"""

import sys

# Check if MTC is available
try:
    from moveit.task_constructor import core, stages
except ImportError as e:
    print("\n" + "="*60)
    print("ERROR: MoveIt Task Constructor not installed!")
    print("="*60)
    print(f"\nImport error: {e}")
    print("\nTo install MTC, run:")
    print("  sudo apt install ros-humble-moveit-task-constructor-*")
    print("\nFor detailed installation instructions, run:")
    print("  ros2 run arm_demos check_mtc.py")
    print("\n" + "="*60)
    sys.exit(1)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped


class MTCSimpleDemo(Node):
    """Simple MTC demonstration node."""

    def __init__(self):
        super().__init__('mtc_simple_demo')

        self.task = None
        self.arm_group_name = "arm"
        self.hand_frame = "wrist_roll_link"

        self.get_logger().info("MTC Simple Demo initialized")

    def create_task(self):
        """Create a simple MTC task."""
        self.task = core.Task("simple_approach_retreat")

        # Load robot model
        self.task.loadRobotModel(self.get_logger())

        # Set task properties
        self.task.setProperty("group", self.arm_group_name)
        self.task.setProperty("ik_frame", self.hand_frame)

        # ==========================
        # Create planners
        # ==========================

        # Sampling planner (OMPL)
        sampling_planner = core.PipelinePlanner()
        sampling_planner.setProperty("goal_joint_tolerance", 1e-5)

        # Cartesian planner
        cartesian_planner = core.CartesianPath()
        cartesian_planner.setMaxVelocityScalingFactor(0.5)  # Slower for safety
        cartesian_planner.setMaxAccelerationScalingFactor(0.5)
        cartesian_planner.setStepSize(0.01)

        # ==========================
        # Define stages
        # ==========================

        # Stage 1: Start from current state
        current_state = stages.CurrentState("current state")
        self.task.add(current_state)

        # Stage 2: Move to home position
        move_to_home = stages.MoveTo("move to home", sampling_planner)
        move_to_home.setGroup(self.arm_group_name)
        move_to_home.setGoal("home")  # Named pose from SRDF
        self.task.add(move_to_home)

        # Stage 3: Move forward (approach)
        approach = stages.MoveRelative("approach", cartesian_planner)
        approach.setGroup(self.arm_group_name)
        approach.setMinMaxDistance(0.10, 0.20)  # Move 10-20cm

        # Set direction (forward in X)
        direction_forward = Vector3Stamped()
        direction_forward.header.frame_id = self.hand_frame
        direction_forward.vector.x = 1.0  # Forward
        approach.setDirection(direction_forward)

        self.task.add(approach)

        # Stage 4: Retreat (move back)
        retreat = stages.MoveRelative("retreat", cartesian_planner)
        retreat.setGroup(self.arm_group_name)
        retreat.setMinMaxDistance(0.10, 0.20)

        # Set direction (backward in X)
        direction_backward = Vector3Stamped()
        direction_backward.header.frame_id = self.hand_frame
        direction_backward.vector.x = -1.0  # Backward
        retreat.setDirection(direction_backward)

        self.task.add(retreat)

        # Stage 5: Return to home
        return_home = stages.MoveTo("return to home", sampling_planner)
        return_home.setGroup(self.arm_group_name)
        return_home.setGoal("home")
        self.task.add(return_home)

        self.get_logger().info("Simple MTC task created")

    def plan_task(self):
        """Plan the task."""
        if self.task is None:
            self.get_logger().error("Task not created")
            return False

        self.get_logger().info("Planning task...")

        try:
            self.task.plan()

            num_solutions = self.task.numSolutions()
            if num_solutions == 0:
                self.get_logger().error("No solutions found!")
                return False

            self.get_logger().info(f"Found {num_solutions} solution(s)")

            # Print the best solution
            best_solution = self.task.solutions()[0]
            self.get_logger().info(f"Best solution cost: {best_solution.cost()}")

            return True

        except Exception as e:
            self.get_logger().error(f"Planning failed: {str(e)}")
            return False

    def execute_task(self):
        """Execute the planned task."""
        if self.task is None or self.task.numSolutions() == 0:
            self.get_logger().error("No solutions to execute")
            return False

        self.get_logger().info("Executing task...")

        try:
            # Execute the best solution
            result = self.task.execute(self.task.solutions()[0])
            self.get_logger().info("Execution completed successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"Execution failed: {str(e)}")
            return False

    def print_task_structure(self):
        """Print the task structure for debugging."""
        if self.task is None:
            self.get_logger().error("Task not created")
            return

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Task Structure:")
        self.get_logger().info("="*50)
        self.get_logger().info(self.task.toString())
        self.get_logger().info("="*50 + "\n")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    demo = MTCSimpleDemo()

    try:
        # Create the task
        demo.create_task()

        # Print task structure
        demo.print_task_structure()

        # Plan the task
        if demo.plan_task():
            # Prompt for execution
            print("\n" + "="*50)
            print("Planning succeeded!")
            print("="*50)
            response = input("\nExecute the task? (y/n): ")

            if response.lower() == 'y':
                demo.execute_task()
            else:
                demo.get_logger().info("Execution skipped by user")
        else:
            demo.get_logger().error("Planning failed. Cannot execute.")

    except KeyboardInterrupt:
        demo.get_logger().info("Interrupted by user")
    except Exception as e:
        demo.get_logger().error(f"Error: {str(e)}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
