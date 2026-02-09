#!/usr/bin/env python3
"""
Planning Scene Publisher for Pick-and-Place Operations

Publishes collision objects (tables and cylinders) to the MoveIt planning scene
for pick-and-place testing and demonstrations.

Usage:
    ros2 run arm_perception planning_scene_publisher.py

Author: LDR Humanoid Arm System
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class PlanningScenePublisher(Node):
    """Publishes collision objects (tables + cylinder) to MoveIt planning scene"""

    def __init__(self):
        super().__init__('planning_scene_publisher')

        # Declare parameters from YAML
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('source_table.name', 'test_table')
        self.declare_parameter('source_table.position.x', -0.30)
        self.declare_parameter('source_table.position.y', 0.35)
        self.declare_parameter('source_table.position.z', 0.2125)
        self.declare_parameter('source_table.dimensions.length', 0.45)
        self.declare_parameter('source_table.dimensions.width', 0.25)
        self.declare_parameter('source_table.dimensions.thickness', 0.425)

        self.declare_parameter('destination_table.name', 'destination_table')
        self.declare_parameter('destination_table.position.x', -0.30)
        self.declare_parameter('destination_table.position.y', -0.35)
        self.declare_parameter('destination_table.position.z', 0.2125)
        self.declare_parameter('destination_table.dimensions.length', 0.45)
        self.declare_parameter('destination_table.dimensions.width', 0.25)
        self.declare_parameter('destination_table.dimensions.thickness', 0.425)

        self.declare_parameter('cylinder.name', 'test_cylinder')
        self.declare_parameter('cylinder.dimensions.height', 0.15)
        self.declare_parameter('cylinder.dimensions.radius', 0.02025)
        self.declare_parameter('cylinder.position_offset.x', 0.0)
        self.declare_parameter('cylinder.position_offset.y', 0.0)

        # Load parameters
        self.frame_id = self.get_parameter('world_frame').value

        # Source table
        self.table_name = self.get_parameter('source_table.name').value
        self.table_x = self.get_parameter('source_table.position.x').value
        self.table_y = self.get_parameter('source_table.position.y').value
        self.table_z = self.get_parameter('source_table.position.z').value
        self.table_length = self.get_parameter('source_table.dimensions.length').value
        self.table_width = self.get_parameter('source_table.dimensions.width').value
        self.table_thickness = self.get_parameter('source_table.dimensions.thickness').value

        # Destination table
        self.dest_table_name = self.get_parameter('destination_table.name').value
        self.dest_table_x = self.get_parameter('destination_table.position.x').value
        self.dest_table_y = self.get_parameter('destination_table.position.y').value
        self.dest_table_z = self.get_parameter('destination_table.position.z').value

        # Cylinder
        self.cylinder_name = self.get_parameter('cylinder.name').value
        self.cylinder_height = self.get_parameter('cylinder.dimensions.height').value
        self.cylinder_radius = self.get_parameter('cylinder.dimensions.radius').value
        self.cylinder_offset_x = self.get_parameter('cylinder.position_offset.x').value
        self.cylinder_offset_y = self.get_parameter('cylinder.position_offset.y').value

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

        self.get_logger().info('Planning Scene Publisher started')
        self.get_logger().info('Publishing source table, destination table, and cylinder to planning scene...')
        self.get_logger().info('Objects will be re-published every 2 seconds to keep them visible')

        # Flag to log only once
        self.logged_initial = False

    def initial_publish(self):
        """Initial publish with logging"""
        self.publish_test_environment()
        if not self.logged_initial:
            # Calculate cylinder position for logging
            table_top_z = self.table_z + (self.table_thickness / 2.0)
            cyl_z = table_top_z + (self.cylinder_height / 2.0)
            cyl_x = self.table_x + self.cylinder_offset_x
            cyl_y = self.table_y + self.cylinder_offset_y

            self.get_logger().info('✅ Planning scene objects published successfully!')
            self.get_logger().info('Objects:')
            self.get_logger().info(
                f'  - Source Table: {self.table_length}m x {self.table_width}m x {self.table_thickness}m '
                f'at ({self.table_x}, {self.table_y}, {self.table_z})'
            )
            self.get_logger().info(
                f'  - Destination Table: {self.table_length}m x {self.table_width}m x {self.table_thickness}m '
                f'at ({self.dest_table_x}, {self.dest_table_y}, {self.dest_table_z})'
            )
            self.get_logger().info(
                f'  - Cylinder: radius={self.cylinder_radius}m, height={self.cylinder_height}m '
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
        table.header.frame_id = self.frame_id
        table.header.stamp = self.get_clock().now().to_msg()

        table.id = self.table_name
        table.operation = CollisionObject.ADD

        # Table dimensions from parameters
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.table_length, self.table_width, self.table_thickness]

        # Table position from parameters
        pose = Pose()
        pose.position.x = self.table_x
        pose.position.y = self.table_y
        pose.position.z = self.table_z
        pose.orientation.w = 1.0  # No rotation

        table.primitives.append(box)
        table.primitive_poses.append(pose)

        self.collision_pub.publish(table)

    def publish_destination_table(self):
        """Publish a destination table as a box"""
        dest_table = CollisionObject()
        dest_table.header = Header()
        dest_table.header.frame_id = self.frame_id
        dest_table.header.stamp = self.get_clock().now().to_msg()

        dest_table.id = self.dest_table_name
        dest_table.operation = CollisionObject.ADD

        # Use same dimensions as main table
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.table_length, self.table_width, self.table_thickness]

        # Destination table position from parameters
        pose = Pose()
        pose.position.x = self.dest_table_x
        pose.position.y = self.dest_table_y
        pose.position.z = self.dest_table_z
        pose.orientation.w = 1.0  # No rotation

        dest_table.primitives.append(box)
        dest_table.primitive_poses.append(pose)

        self.collision_pub.publish(dest_table)

    def publish_cylinder(self):
        """Publish a cylinder object to grasp (automatically positioned on table)"""
        cylinder = CollisionObject()
        cylinder.header = Header()
        cylinder.header.frame_id = self.frame_id
        cylinder.header.stamp = self.get_clock().now().to_msg()

        cylinder.id = self.cylinder_name
        cylinder.operation = CollisionObject.ADD

        # Cylinder dimensions from parameters
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [self.cylinder_height, self.cylinder_radius]

        # Calculate cylinder position:
        # 1. Table top Z = table_z + table_thickness/2
        # 2. Cylinder center Z = table_top_z + cylinder_height/2
        # 3. X, Y = TABLE position + offsets
        table_top_z = self.table_z + (self.table_thickness / 2.0)
        cylinder_z = table_top_z + (self.cylinder_height / 2.0)

        pose = Pose()
        pose.position.x = self.table_x + self.cylinder_offset_x
        pose.position.y = self.table_y + self.cylinder_offset_y
        pose.position.z = cylinder_z
        pose.orientation.w = 1.0  # Upright orientation

        cylinder.primitives.append(cyl)
        cylinder.primitive_poses.append(pose)

        self.collision_pub.publish(cylinder)

    def clear_environment(self):
        """Clear all test objects from planning scene"""
        for obj_id in [self.table_name, self.dest_table_name, self.cylinder_name]:
            remove_obj = CollisionObject()
            remove_obj.header = Header()
            remove_obj.header.frame_id = self.frame_id
            remove_obj.header.stamp = self.get_clock().now().to_msg()
            remove_obj.id = obj_id
            remove_obj.operation = CollisionObject.REMOVE
            self.collision_pub.publish(remove_obj)

        self.get_logger().info('Planning scene objects cleared')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningScenePublisher()

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
