#!/usr/bin/env python3
"""
Test Environment Publisher for Gripper Testing

Publishes a table and a cylinder to the MoveIt planning scene for testing
gripper pick and place operations.

Usage:
    ros2 run arm_perception test_environment_publisher.py

Author: LDR Humanoid Arm System
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class TestEnvironmentPublisher(Node):
    """Publishes test objects (table + cylinder) to MoveIt planning scene"""

    def __init__(self):
        super().__init__('test_environment_publisher')

        # Publisher for collision objects
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        # Publish objects periodically to ensure they stay in planning scene
        # Initial publish after 1 second, then every 2 seconds
        self.create_timer(1.0, self.initial_publish)
        self.create_timer(2.0, self.publish_test_environment)

        self.get_logger().info('Test Environment Publisher started')
        self.get_logger().info('Publishing table and cylinder to planning scene...')
        self.get_logger().info('Objects will be re-published every 2 seconds to keep them visible')

        # Flag to log only once
        self.logged_initial = False

    def initial_publish(self):
        """Initial publish with logging"""
        self.publish_test_environment()
        if not self.logged_initial:
            self.get_logger().info('✅ Test environment published successfully!')
            self.get_logger().info('Objects:')
            self.get_logger().info('  - Table: 0.8m x 0.6m x 0.02m at (0.5, 0.0, 0.4)')
            self.get_logger().info('  - Cylinder: radius=0.03m, height=0.15m at (0.5, 0.0, 0.485)')
            self.logged_initial = True

    def publish_test_environment(self):
        """Publish table and cylinder objects (called periodically)"""
        # Publish table
        self.publish_table()

        # Publish cylinder
        self.publish_cylinder()

    def publish_table(self):
        """Publish a table as a box"""
        table = CollisionObject()
        table.header = Header()
        table.header.frame_id = 'base_link'
        table.header.stamp = self.get_clock().now().to_msg()

        table.id = 'test_table'
        table.operation = CollisionObject.ADD

        # Table dimensions (0.8m x 0.6m x 0.02m thick)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 0.6, 0.02]  # X, Y, Z (length, width, thickness)

        # Table position (50cm in front, at 40cm height, centered)
        pose = Pose()
        pose.position.x = 0.5   # 50cm in front of robot
        pose.position.y = 0.0   # Centered
        pose.position.z = 0.4   # 40cm height (top surface at 41cm)
        pose.orientation.w = 1.0  # No rotation

        table.primitives.append(box)
        table.primitive_poses.append(pose)

        self.collision_pub.publish(table)

    def publish_cylinder(self):
        """Publish a cylinder object to grasp"""
        cylinder = CollisionObject()
        cylinder.header = Header()
        cylinder.header.frame_id = 'base_link'
        cylinder.header.stamp = self.get_clock().now().to_msg()

        cylinder.id = 'test_cylinder'
        cylinder.operation = CollisionObject.ADD

        # Cylinder dimensions (radius=3cm, height=15cm - typical can size)
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.15, 0.03]  # [height, radius]

        # Cylinder position (on top of table, centered)
        # Table top is at z=0.4 + 0.01 = 0.41m
        # Cylinder center at 0.41 + 0.15/2 = 0.485m
        pose = Pose()
        pose.position.x = 0.5      # Same X as table (on table)
        pose.position.y = 0.0      # Centered on table
        pose.position.z = 0.485    # Standing on table (0.41 + 0.075)
        pose.orientation.w = 1.0   # Upright

        cylinder.primitives.append(cyl)
        cylinder.primitive_poses.append(pose)

        self.collision_pub.publish(cylinder)

    def clear_environment(self):
        """Clear all test objects from planning scene"""
        for obj_id in ['test_table', 'test_cylinder']:
            remove_obj = CollisionObject()
            remove_obj.header = Header()
            remove_obj.header.frame_id = 'base_link'
            remove_obj.header.stamp = self.get_clock().now().to_msg()
            remove_obj.id = obj_id
            remove_obj.operation = CollisionObject.REMOVE
            self.collision_pub.publish(remove_obj)

        self.get_logger().info('Test environment cleared')


def main(args=None):
    rclpy.init(args=args)
    node = TestEnvironmentPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.clear_environment()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
