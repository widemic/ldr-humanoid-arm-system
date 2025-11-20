#!/usr/bin/env python3
"""
Simple MTC Movement Demo

A minimal example demonstrating basic MTC usage with the 5-DOF arm.
This task simply moves the arm through a sequence of poses.
"""

import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
import sys


class SimpleMoveTask(Node):
    def __init__(self):
        super().__init__('simple_move_demo')

        # Task setup
        self.task = core.Task()
        self.task.name = "simple_move_task"

        # Planning group
        self.arm_group = "arm"

        self.get_logger().info("Initializing Simple Move Task...")

    def setup_task(self):
        """Configure a simple movement task"""

        # 1. Start from current state
        current_state = stages.CurrentState("current state")
        self.task.add(current_state)

        # 2. Move to home position
        move_to_home = stages.MoveTo("move to home", self.arm_group)
        move_to_home.setGoal("home")  # Uses predefined pose from SRDF
        self.task.add(move_to_home)

        # 3. Connect to next pose
        connect = stages.Connect(
            "connect",
            stages.Connect.GroupPlannerVector([
                (self.arm_group, "RRTConnect")
            ])
        )
        connect.setTimeout(5.0)
        self.task.add(connect)

        # 4. Move to another position (can add custom joint values)
        # Example: slightly different configuration
        # move_to_custom = stages.MoveTo("move to custom", self.arm_group)
        # move_to_custom.setGoal([0.5, 1.0, -0.5, 1.2, 0.3])  # Joint positions
        # self.task.add(move_to_custom)

        # 5. Return to home
        return_home = stages.MoveTo("return home", self.arm_group)
        return_home.setGoal("home")
        self.task.add(return_home)

        self.get_logger().info("Task pipeline configured with {} stages".format(
            len(self.task.stages)))

    def plan(self):
        """Plan the task"""
        self.get_logger().info("Planning task...")

        try:
            success = self.task.plan()
            if success:
                num_solutions = len(self.task.solutions)
                self.get_logger().info(
                    f"Planning succeeded! Found {num_solutions} solution(s)")
                return True
            else:
                self.get_logger().error("Planning failed!")
                return False
        except Exception as e:
            self.get_logger().error(f"Planning error: {str(e)}")
            return False

    def execute(self):
        """Execute the planned task"""
        self.get_logger().info("Executing task...")

        try:
            success = self.task.execute()
            if success:
                self.get_logger().info("Task execution succeeded!")
                return True
            else:
                self.get_logger().error("Task execution failed!")
                return False
        except Exception as e:
            self.get_logger().error(f"Execution error: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)

    simple_move = SimpleMoveTask()

    # Setup the task
    simple_move.setup_task()

    # Plan the task
    if simple_move.plan():
        simple_move.get_logger().info("Plan successful! Executing...")
        simple_move.execute()
    else:
        simple_move.get_logger().error("Planning failed. Exiting.")
        simple_move.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Keep node alive briefly to see results
    rclpy.spin_once(simple_move, timeout_sec=1.0)

    simple_move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
