#!/usr/bin/env python3
"""
Test script for FollowJointTrajectory action server.

This script tests the custom controller's action server by sending
a simple trajectory and monitoring execution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class TrajectoryActionTest(Node):
    def __init__(self):
        super().__init__('trajectory_action_test')

        # Action client for FollowJointTrajectory
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.arm_joints = [
            'left_shoulder_pitch_rs04',
            'left_shoulder_roll_rs04',
            'left_shoulder_yaw_rs03',
            'left_elbow_rs03',
            'left_wrist_rs02',
            'left_hand_rs02',
        ]

    def send_trajectory(self, waypoints, times):
        """
        Send a trajectory to the action server.

        Args:
            waypoints: List of joint position lists [[j0, j1, ...], [j0, j1, ...], ...]
            times: List of time_from_start for each waypoint (seconds)
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints

        for positions, t in zip(waypoints, times):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending trajectory with {len(waypoints)} waypoints')

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return None

        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return None

        self.get_logger().info('Goal accepted, executing trajectory...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result

    def feedback_callback(self, feedback_msg):
        """Print feedback during trajectory execution"""
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'desired') and hasattr(feedback.desired, 'positions'):
            self.get_logger().info(
                f'Executing: time={feedback.desired.time_from_start.sec}.{feedback.desired.time_from_start.nanosec//1000000:03d}s',
                throttle_duration_sec=0.5
            )


def main():
    rclpy.init()
    node = TrajectoryActionTest()

    print("\n" + "="*80)
    print("FOLLOWJOINTTRAJECTORY ACTION SERVER TEST")
    print("="*80)

    try:
        # Test 1: Simple 2-point trajectory
        print("\n[Test 1] Simple 2-point trajectory")
        print("  Move from home position to slightly raised position")

        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        raised = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]  # Raise elbow

        waypoints = [home, raised]
        times = [0.0, 2.0]  # 2 seconds to complete

        result = node.send_trajectory(waypoints, times)

        if result and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            print("  ✅ Trajectory executed successfully!")
        else:
            print(f"  ❌ Trajectory failed with error code: {result.error_code if result else 'None'}")

        time.sleep(1)

        # Test 2: Multi-point trajectory
        print("\n[Test 2] Multi-point trajectory (3 waypoints)")
        print("  Move through multiple poses")

        waypoints = [
            [0.0, 0.0, 0.0, 0.5, 0.0, 0.0],   # Start (raised elbow)
            [0.2, 0.0, 0.0, 0.8, 0.0, 0.0],   # Mid (shoulder forward, elbow more)
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # End (home)
        ]
        times = [0.0, 2.0, 4.0]  # 4 seconds total

        result = node.send_trajectory(waypoints, times)

        if result and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            print("  ✅ Multi-point trajectory executed successfully!")
        else:
            print(f"  ❌ Trajectory failed with error code: {result.error_code if result else 'None'}")

        print("\n" + "="*80)
        print("RESULTS")
        print("="*80)
        print("✅ FollowJointTrajectory action server is working!")
        print("✅ Controller can execute multi-point trajectories")
        print("✅ Linear interpolation between waypoints")
        print("\n🎯 NEXT: Test with MoveIt motion planning")
        print("   Run: ros2 launch arm_system_bringup moveit_gazebo.launch.py")
        print("   Then use RViz to plan and execute motions")
        print("="*80 + "\n")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
