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

    # ========== CONFIGURATION PARAMETERS ==========

    # Table dimensions (X, Y, Z in meters)
    TABLE_LENGTH = 0.5      # Front-to-back dimension
    TABLE_WIDTH = 0.3       # Left-to-right dimension
    TABLE_THICKNESS = 0.02  # Height/thickness

    # Table position (center of table)
    TABLE_X = -0.3         # Distance from robot base (closer: was -0.5)
    TABLE_Y = 0.35            # Lateral position (more centered: was 0.4)
    TABLE_Z = 0.9            # Height of table center (higher: was 0.9)

    # Destination Table position (center of table)
    DEST_TABLE_X = -0.7         # Distance from robot base (farther from robot)
    DEST_TABLE_Y = 0.0          # Lateral position (centered)
    DEST_TABLE_Z = 0.9          # Height of table center (same as source table)

    # Cylinder dimensions
    CYLINDER_RADIUS = 0.03  # Radius (3cm)
    CYLINDER_HEIGHT = 0.15  # Height (15cm)

    # Cylinder position on table (X, Y relative to table center)
    CYLINDER_OFFSET_X = 0.0 # X offset from table center (0 = centered)
    CYLINDER_OFFSET_Y = 0.0 # Y offset from table center (0 = centered)

    # Reference frame
    FRAME_ID = 'base_link'

    # ==============================================

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
        self.get_logger().info('Publishing source table, destination table, and cylinder to planning scene...')
        self.get_logger().info('Objects will be re-published every 2 seconds to keep them visible')

        # Flag to log only once
        self.logged_initial = False

    def initial_publish(self):
        """Initial publish with logging"""
        self.publish_test_environment()
        if not self.logged_initial:
            # Calculate cylinder position for logging
            table_top_z = self.TABLE_Z + (self.TABLE_THICKNESS / 2.0)
            cyl_z = table_top_z + (self.CYLINDER_HEIGHT / 2.0)
            cyl_x = self.TABLE_X + self.CYLINDER_OFFSET_X
            cyl_y = self.TABLE_Y + self.CYLINDER_OFFSET_Y

            self.get_logger().info('✅ Test environment published successfully!')
            self.get_logger().info('Objects:')
            self.get_logger().info(
                f'  - Source Table: {self.TABLE_LENGTH}m x {self.TABLE_WIDTH}m x {self.TABLE_THICKNESS}m '
                f'at ({self.TABLE_X}, {self.TABLE_Y}, {self.TABLE_Z})'
            )
            self.get_logger().info(
                f'  - Destination Table: {self.TABLE_LENGTH}m x {self.TABLE_WIDTH}m x {self.TABLE_THICKNESS}m '
                f'at ({self.DEST_TABLE_X}, {self.DEST_TABLE_Y}, {self.DEST_TABLE_Z})'
            )
            self.get_logger().info(
                f'  - Cylinder: radius={self.CYLINDER_RADIUS}m, height={self.CYLINDER_HEIGHT}m '
                f'at ({cyl_x:.3f}, {cyl_y:.3f}, {cyl_z:.3f})'
            )
            self.logged_initial = True

    def publish_test_environment(self):
        """Publish table, destination table, and cylinder objects (called periodically)"""
        # Publish source table
        self.publish_table()

        # Publish destination table
        self.publish_destination_table()

        # Publish cylinder
        self.publish_cylinder()

    def publish_table(self):
        """Publish a table as a box"""
        table = CollisionObject()
        table.header = Header()
        table.header.frame_id = self.FRAME_ID
        table.header.stamp = self.get_clock().now().to_msg()

        table.id = 'test_table'
        table.operation = CollisionObject.ADD

        # Table dimensions from class variables
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.TABLE_LENGTH, self.TABLE_WIDTH, self.TABLE_THICKNESS]

        # Table position from class variables
        pose = Pose()
        pose.position.x = self.TABLE_X
        pose.position.y = self.TABLE_Y
        pose.position.z = self.TABLE_Z
        pose.orientation.w = 1.0  # No rotation

        table.primitives.append(box)
        table.primitive_poses.append(pose)

        self.collision_pub.publish(table)

    def publish_destination_table(self):
        """Publish a destination table as a box"""
        dest_table = CollisionObject()
        dest_table.header = Header()
        dest_table.header.frame_id = self.FRAME_ID
        dest_table.header.stamp = self.get_clock().now().to_msg()

        dest_table.id = 'destination_table'
        dest_table.operation = CollisionObject.ADD

        # Use same dimensions as main table
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.TABLE_LENGTH, self.TABLE_WIDTH, self.TABLE_THICKNESS]

        # Destination table position from class variables
        pose = Pose()
        pose.position.x = self.DEST_TABLE_X
        pose.position.y = self.DEST_TABLE_Y
        pose.position.z = self.DEST_TABLE_Z
        pose.orientation.w = 1.0  # No rotation

        dest_table.primitives.append(box)
        dest_table.primitive_poses.append(pose)

        self.collision_pub.publish(dest_table)

    def publish_cylinder(self):
        """Publish a cylinder object to grasp (automatically positioned on table)"""
        cylinder = CollisionObject()
        cylinder.header = Header()
        cylinder.header.frame_id = self.FRAME_ID
        cylinder.header.stamp = self.get_clock().now().to_msg()

        cylinder.id = 'test_cylinder'
        cylinder.operation = CollisionObject.ADD

        # Cylinder dimensions from class variables
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [self.CYLINDER_HEIGHT, self.CYLINDER_RADIUS]

        # Calculate cylinder position:
        # 1. Table top Z = TABLE_Z + TABLE_THICKNESS/2
        # 2. Cylinder center Z = table_top_z + CYLINDER_HEIGHT/2
        # 3. X, Y = TABLE position + offsets
        table_top_z = self.TABLE_Z + (self.TABLE_THICKNESS / 2.0)
        cylinder_z = table_top_z + (self.CYLINDER_HEIGHT / 2.0)

        pose = Pose()
        pose.position.x = self.TABLE_X + self.CYLINDER_OFFSET_X
        pose.position.y = self.TABLE_Y + self.CYLINDER_OFFSET_Y
        pose.position.z = cylinder_z
        pose.orientation.w = 1.0  # Upright orientation

        cylinder.primitives.append(cyl)
        cylinder.primitive_poses.append(pose)

        self.collision_pub.publish(cylinder)

    def clear_environment(self):
        """Clear all test objects from planning scene"""
        for obj_id in ['test_table', 'destination_table', 'test_cylinder']:
            remove_obj = CollisionObject()
            remove_obj.header = Header()
            remove_obj.header.frame_id = self.FRAME_ID
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
