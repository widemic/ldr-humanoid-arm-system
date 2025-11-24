#!/usr/bin/env python3
"""
Test workspace reachability for the arm

This script tests if specific positions are reachable by the arm
by attempting to compute IK solutions.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import math


class WorkspaceTest(Node):
    def __init__(self):
        super().__init__('workspace_test')

        # IK service client
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service()
        self.get_logger().info('✓ IK service available')

    def test_position(self, x, y, z, description=""):
        """Test if a position is reachable (position-only IK)"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2

        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # ANY orientation - let IK solver decide
        # Using identity quaternion, but we'll request position-only IK
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        request.ik_request.pose_stamped = pose
        request.ik_request.ik_link_name = 'left_palm'

        # CRITICAL: Request position-only IK (ignore orientation)
        # This is what MoveIt uses internally when orientation constraints are omitted
        # Set all orientation tolerances to 2*pi to effectively disable orientation checking
        from moveit_msgs.msg import Constraints, OrientationConstraint
        constraints = Constraints()
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose.header
        orient_constraint.link_name = 'left_palm'
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 6.28  # 2*pi - any orientation
        orient_constraint.absolute_y_axis_tolerance = 6.28
        orient_constraint.absolute_z_axis_tolerance = 6.28
        orient_constraint.weight = 0.01
        constraints.orientation_constraints.append(orient_constraint)
        request.ik_request.constraints = constraints

        # Call IK service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.done():
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                self.get_logger().info(
                    f'✓ REACHABLE: ({x:.3f}, {y:.3f}, {z:.3f}) {description}'
                )
                return True
            else:
                self.get_logger().warn(
                    f'✗ NOT REACHABLE: ({x:.3f}, {y:.3f}, {z:.3f}) '
                    f'{description} (error: {response.error_code.val})'
                )
                return False
        else:
            self.get_logger().error(f'✗ IK TIMEOUT: ({x:.3f}, {y:.3f}, {z:.3f}) {description}')
            return False


def main():
    rclpy.init()
    tester = WorkspaceTest()

    # Test positions from pick_and_place (MUST match test_environment_publisher.py)
    TABLE_X = -0.35
    TABLE_Y = 0.3
    TABLE_Z = 0.95
    TABLE_THICKNESS = 0.02
    CYLINDER_HEIGHT = 0.15

    table_top_z = TABLE_Z + (TABLE_THICKNESS / 2.0)
    cylinder_z = table_top_z + (CYLINDER_HEIGHT / 2.0)

    print("\n" + "="*60)
    print("WORKSPACE REACHABILITY TEST")
    print("="*60)
    print(f"Table: ({TABLE_X}, {TABLE_Y}, {TABLE_Z})")
    print(f"Cylinder center: ({TABLE_X}, {TABLE_Y}, {cylinder_z:.3f})")
    print("="*60 + "\n")

    # Test various heights
    test_positions = [
        (TABLE_X, TABLE_Y, cylinder_z + 0.20, "Pre-grasp +20cm"),
        (TABLE_X, TABLE_Y, cylinder_z + 0.15, "Pre-grasp +15cm"),
        (TABLE_X, TABLE_Y, cylinder_z + 0.10, "Pre-grasp +10cm"),
        (TABLE_X, TABLE_Y, cylinder_z + 0.05, "Grasp approach +5cm"),
        (TABLE_X, TABLE_Y, cylinder_z, "Cylinder center (grasp)"),
        (TABLE_X, TABLE_Y, table_top_z, "Table surface"),
        # Test different X positions
        (TABLE_X - 0.1, TABLE_Y, cylinder_z + 0.10, "10cm closer, +10cm high"),
        (TABLE_X + 0.1, TABLE_Y, cylinder_z + 0.10, "10cm farther, +10cm high"),
        # Test different Y positions
        (TABLE_X, TABLE_Y - 0.1, cylinder_z + 0.10, "10cm left, +10cm high"),
        (TABLE_X, TABLE_Y + 0.1, cylinder_z + 0.10, "10cm right, +10cm high"),
    ]

    reachable = 0
    for x, y, z, desc in test_positions:
        if tester.test_position(x, y, z, desc):
            reachable += 1

    print("\n" + "="*60)
    print(f"RESULTS: {reachable}/{len(test_positions)} positions reachable")
    print("="*60 + "\n")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
