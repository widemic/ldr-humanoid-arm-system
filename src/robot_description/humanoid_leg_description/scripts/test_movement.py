#!/usr/bin/env python3
"""
Simple test script to move the robot legs in Gazebo.
Sends trajectory commands to the leg_controller.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class LegMovementTest(Node):
    def __init__(self):
        super().__init__('leg_movement_test')
        
        # Create publisher for leg_controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/leg_controller/joint_trajectory',
            10
        )
        
        # Wait for publisher to be ready
        self.get_logger().info('Waiting for subscribers...')
        rclpy.spin_once(self, timeout_sec=2.0)
        
        self.get_logger().info('Starting leg movement test...')
        
        # Create trajectory message
        self.send_trajectory()
        
    def send_trajectory(self):
        """Send a simple trajectory to move the legs."""
        msg = JointTrajectory()
        
        # Joint names (all 12 leg joints)
        msg.joint_names = [
            'left_hip_pitch',
            'left_hip_roll',
            'left_hip_yaw',
            'left_knee',
            'left_ankle',
            'left_foot',
            'right_hip_pitch',
            'right_hip_roll',
            'right_hip_yaw',
            'right_knee',
            'right_ankle',
            'right_foot'
        ]
        
        # Create two points for a simple motion
        # Point 1: Move to position (after 2 seconds)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.3, 0.0, 0.0, 0.5, 0.0, 0.0,  # Left leg
                           0.3, 0.0, 0.0, 0.5, 0.0, 0.0]  # Right leg
        point1.time_from_start = Duration(sec=2, nanosec=0)
        
        # Point 2: Return to zero (after 4 seconds)
        point2 = JointTrajectoryPoint()
        point2.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Left leg
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Right leg
        point2.time_from_start = Duration(sec=4, nanosec=0)
        
        msg.points = [point1, point2]
        
        # Publish trajectory
        self.publisher.publish(msg)
        self.get_logger().info('Trajectory sent! Check Gazebo for movement.')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LegMovementTest()
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('Test complete!')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
