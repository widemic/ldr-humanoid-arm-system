#!/usr/bin/env python3
"""
Test script for gripper controller
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import time


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        self._action_client = ActionClient(self, GripperCommand, 'hand_controller/gripper_cmd')

    def send_goal(self, position, max_effort=10.0):
        """Send a gripper goal

        Args:
            position: Target position in meters
                     -0.033 = fully closed
                      0.0 = fully open
            max_effort: Maximum effort in N
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'Sending gripper goal: position={position:.4f}m, effort={max_effort}N')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f'Result: position={result.position:.4f}, effort={result.effort:.2f}, '
            f'reached_goal={result.reached_goal}, stalled={result.stalled}'
        )

        return True


def main():
    rclpy.init()
    tester = GripperTester()

    try:
        tester.get_logger().info('Running gripper test cycle: open -> close -> open')

        # Test 1: Open gripper (position = 0.0)
        tester.send_goal(position=0.0, max_effort=10.0)
        tester.get_logger().info('Waiting 2 seconds...')
        time.sleep(2)

        # Test 2: Close gripper (position = -0.030, slightly less than full -0.033)
        tester.send_goal(position=-0.030, max_effort=10.0)
        tester.get_logger().info('Waiting 2 seconds...')
        time.sleep(2)

        # Test 3: Open gripper again
        tester.send_goal(position=0.0, max_effort=10.0)

        tester.get_logger().info('Test cycle complete!')

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
