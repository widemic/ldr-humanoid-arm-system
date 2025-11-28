#!/usr/bin/env python3
"""
MTC Solution Executor

This node subscribes to /solution topic and automatically executes
the MTC solution by calling the /execute_task_solution action.

Usage:
    ros2 run arm_perception mtc_solution_executor.py

Workflow:
    1. Run the MTC pick and place planner: ros2 launch arm_perception mtc_pick_place.launch.py
    2. This executor automatically receives the solution and executes it
    3. Monitor execution progress in terminal
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_task_constructor_msgs.msg import Solution
from moveit_task_constructor_msgs.action import ExecuteTaskSolution


class MTCSolutionExecutor(Node):
    def __init__(self):
        super().__init__('mtc_solution_executor')

        # Action client for execution
        self._action_client = ActionClient(
            self,
            ExecuteTaskSolution,
            '/execute_task_solution'
        )

        # Subscriber for solutions
        self._solution_sub = self.create_subscription(
            Solution,
            '/solution',
            self._solution_callback,
            10
        )

        self._executing = False

        self.get_logger().info('========================================')
        self.get_logger().info('MTC Solution Executor Ready')
        self.get_logger().info('Waiting for solutions on /solution...')
        self.get_logger().info('========================================')

    def _solution_callback(self, solution_msg):
        """Receives solution from /solution and executes it"""
        if self._executing:
            self.get_logger().warn('Already executing a solution, ignoring new solution')
            return

        self.get_logger().info('========================================')
        self.get_logger().info('Received MTC solution!')
        self.get_logger().info(f'Task ID: {solution_msg.task_id}')
        self.get_logger().info(f'Sub-trajectories: {len(solution_msg.sub_trajectory)}')
        self.get_logger().info('Sending execution request...')
        self.get_logger().info('========================================')

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /execute_task_solution not available!')
            self.get_logger().error('Make sure move_group is running with ExecuteTaskSolutionCapability')
            return

        # Create goal
        goal_msg = ExecuteTaskSolution.Goal()
        goal_msg.solution = solution_msg

        # Send goal
        self._executing = True
        self.get_logger().info('Executing solution...')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server!')
            self._executing = False
            return

        self.get_logger().info('Goal accepted by action server')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Receive execution feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Executing sub-trajectory {feedback.sub_no}/{feedback.sub_id}'
        )

    def _result_callback(self, future):
        """Handle execution result"""
        result = future.result().result
        self._executing = False

        self.get_logger().info('========================================')

        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✅ Execution completed successfully!')
            self.get_logger().info(f'Result: {result.error_code.message}')
        else:
            self.get_logger().error(f'❌ Execution failed!')
            self.get_logger().error(f'Error code: {result.error_code.val}')
            self.get_logger().error(f'Error message: {result.error_code.message}')
            self.get_logger().error(f'Source: {result.error_code.source}')

        self.get_logger().info('========================================')
        self.get_logger().info('Ready for next solution...')


def main(args=None):
    rclpy.init(args=args)
    executor = MTCSolutionExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
