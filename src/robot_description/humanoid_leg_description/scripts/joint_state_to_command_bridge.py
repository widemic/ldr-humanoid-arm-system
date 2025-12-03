#!/usr/bin/env python3
"""
Bridge node that forwards joint_states to leg_controller commands.
This allows joint_state_publisher_gui to control the robot in Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointStateToCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_state_to_command_bridge')
        
        # Joint names for leg controller
        self.joint_names = [
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
        
        # Subscribe to joint_states (from joint_state_publisher_gui)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish to leg_controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/leg_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Bridge node started. Forwarding /joint_states to /leg_controller/joint_trajectory')
        
    def joint_state_callback(self, msg: JointState):
        """Convert joint states to trajectory commands."""
        
        # Extract positions for leg joints
        positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            else:
                # If joint not in message, use 0.0
                positions.append(0.0)
        
        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Create single point with current positions
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 second
        
        traj_msg.points = [point]
        
        # Publish trajectory
        self.publisher.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JointStateToCommandBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
