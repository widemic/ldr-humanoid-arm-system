#!/usr/bin/env python3
"""
Simple Motion Planner - Clean interface to control the arm in Gazebo.

This provides a simple API for controlling the robot arm from planning code.
No complex ROS2 control knowledge needed - just positions and trajectories.

Example:
    planner = MotionPlanner()
    planner.move_to([0, 1, 0, 1, 0])  # Move to position
    planner.home()                     # Go home
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import threading


class MotionPlanner(Node):
    """Simple motion planner for controlling the arm."""

    # Predefined poses
    POSES = {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0],
        'ready': [0.0, 1.0, 0.0, 1.0, 0.0],
        'vertical': [0.0, 1.57, 0.0, 1.57, 0.0],
    }

    def __init__(self):
        super().__init__('motion_planner')

        # 5-DOF arm joints
        self.joints = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        # Action client
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Current state
        self._current_pos = None
        self._lock = threading.Lock()

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self._state_callback,
            10
        )

        # Wait for action server
        self.get_logger().info('Waiting for controller...')
        if self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Connected!')
        else:
            self.get_logger().error('Controller not available!')

    def _state_callback(self, msg):
        """Update current joint positions."""
        with self._lock:
            pos = {}
            for i, name in enumerate(msg.name):
                if name in self.joints:
                    pos[name] = msg.position[i]
            if len(pos) == 5:
                self._current_pos = [pos[j] for j in self.joints]

    def get_position(self):
        """Get current joint positions."""
        with self._lock:
            return self._current_pos.copy() if self._current_pos else None

    def move_to(self, positions, duration=2.0):
        """
        Move to joint positions.

        Args:
            positions: List of 5 joint angles [rad]
            duration: Time to complete motion [sec]

        Returns:
            bool: Success
        """
        if len(positions) != 5:
            self.get_logger().error(f'Need 5 positions, got {len(positions)}')
            return False

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        # Single point trajectory
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * 5
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )

        goal.trajectory.points = [point]

        # Send
        self.get_logger().info(f'Moving to {positions}')
        future = self._client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().accepted:
            self.get_logger().info('Moving...')
            # Wait for completion
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(
                self,
                result_future,
                timeout_sec=duration + 2.0
            )
            return result_future.result().status == 4  # SUCCEEDED
        else:
            self.get_logger().error('Goal rejected')
            return False

    def move_trajectory(self, waypoints):
        """
        Execute multi-point trajectory.

        Args:
            waypoints: List of (positions, time) tuples
                positions: [5 joint angles in rad]
                time: time from start [sec]

        Example:
            planner.move_trajectory([
                ([0, 1, 0, 1, 0], 2.0),
                ([0.5, 1.5, -0.5, 1.5, 0.5], 4.0),
                ([0, 0, 0, 0, 0], 6.0)
            ])
        """
        if not waypoints:
            return False

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        # Add waypoints
        for positions, time in waypoints:
            if len(positions) != 5:
                self.get_logger().error(f'Need 5 positions, got {len(positions)}')
                return False

            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.velocities = [0.0] * 5
            point.time_from_start = Duration(
                sec=int(time),
                nanosec=int((time % 1) * 1e9)
            )
            goal.trajectory.points.append(point)

        # Send
        self.get_logger().info(f'Executing {len(waypoints)} waypoints')
        future = self._client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().accepted:
            # Wait for completion
            result_future = future.result().get_result_async()
            max_time = waypoints[-1][1]
            rclpy.spin_until_future_complete(
                self,
                result_future,
                timeout_sec=max_time + 2.0
            )
            return result_future.result().status == 4
        else:
            self.get_logger().error('Goal rejected')
            return False

    def home(self):
        """Move to home position."""
        return self.move_to(self.POSES['home'])

    def ready(self):
        """Move to ready position."""
        return self.move_to(self.POSES['ready'])


def main():
    """Example usage."""
    rclpy.init()
    planner = MotionPlanner()

    # Wait for state
    import time
    time.sleep(1.0)

    try:
        # Show current position
        pos = planner.get_position()
        if pos:
            planner.get_logger().info(f'Current: {pos}')

        # Example motions
        planner.get_logger().info('=== Demo ===')

        planner.ready()
        time.sleep(0.5)

        planner.move_to([0.5, 1.5, -0.5, 1.5, 0.5])
        time.sleep(0.5)

        planner.home()

        planner.get_logger().info('=== Done ===')

    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
