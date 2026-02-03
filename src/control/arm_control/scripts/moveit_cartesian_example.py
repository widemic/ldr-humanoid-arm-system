#!/usr/bin/env python3
"""
Move arm to (x,y,z) using MoveIt IK.

Simple: MoveIt calculates joint angles → send to controller
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped


class CartesianMover(Node):
    def __init__(self):
        super().__init__('cartesian_mover')

        # IK service
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.ik.wait_for_service()

        # Joint command publisher
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def move_to(self, x, y, z):
        """Move to target position."""

        # Ask MoveIt for joint angles
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = pose

        future = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().error_code.val != 1:
            print("✗ IK failed - position unreachable")
            return False

        # Send to controller
        solution = future.result().solution.joint_state

        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pitch_joint", "shoulder_roll_joint",
                           "shoulder_yaw_joint", "elbow_pitch_joint", "elbow_yaw_joint"]

        point = JointTrajectoryPoint()
        point.positions = list(solution.position[:5])
        point.time_from_start.sec = 5
        msg.points = [point]

        self.pub.publish(msg)
        print("✓ Command sent")
        return True


def main():
    rclpy.init()

    mover = CartesianMover()

    x, y, z = -0.5, 0.4, 0.8
    print(f"Moving to: x={x}, y={y}, z={z}")

    mover.move_to(x, y, z)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
