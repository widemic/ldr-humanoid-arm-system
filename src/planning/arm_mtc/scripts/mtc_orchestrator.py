#!/usr/bin/env python3
"""
MTC Orchestrator - Pick and Place Multiple Objects

This script orchestrates multiple pick-and-place operations by:
1. Spawning objects in the planning scene
2. Calling the MTC node with appropriate parameters for each object
3. Supporting stacking operations

Usage:
    ros2 run arm_mtc mtc_orchestrator.py
    
Or with custom scene:
    ros2 run arm_mtc mtc_orchestrator.py --scene stack_demo
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
import subprocess
import time
import sys
import argparse


class MTCOrchestrator(Node):
    def __init__(self):
        super().__init__('mtc_orchestrator')
        
        # Publisher for planning scene
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        
        self.get_logger().info('MTC Orchestrator initialized')
    
    def spawn_cylinder(self, object_id: str, radius: float, height: float,
                       x: float, y: float, z: float):
        """Spawn a cylinder in the planning scene."""
        
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]  # height, radius
        obj.primitives.append(primitive)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        
        # Publish to planning scene
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)
        
        self.get_logger().info(
            f"Spawned '{object_id}': r={radius:.4f}m, h={height:.3f}m at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        time.sleep(0.5)  # Wait for scene update
    
    def spawn_table(self, x: float = 0.3, y: float = 0.0, z: float = 0.6,
                    width: float = 0.5, depth: float = 0.5, height: float = 0.2):
        """Spawn a table in the planning scene."""
        
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = 'table'
        obj.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [width, depth, height]
        obj.primitives.append(primitive)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)
        
        self.get_logger().info(f"Spawned table at ({x:.3f}, {y:.3f}, {z:.3f})")
        time.sleep(0.5)
    
    def spawn_round_table(self, table_id: str, x: float, y: float, z: float,
                          diameter: float = 0.25, leg_height: float = 0.025):
        """Spawn a round table (cylinder) in the planning scene.
        
        Args:
            table_id: Unique ID for the table
            x, y, z: Position of the table TOP surface center
            diameter: Table top diameter (default 25cm)
            leg_height: Height of the table/leg (default 2.5cm)
        """
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = table_id
        obj.operation = CollisionObject.ADD
        
        # Table is a cylinder
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [leg_height, diameter / 2.0]  # [height, radius]
        obj.primitives.append(primitive)
        
        # Position: z is at table center, so offset down by half height
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z - leg_height / 2.0  # Center of cylinder
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)
        
        self.get_logger().info(
            f"Spawned round table '{table_id}': d={diameter:.2f}m, h={leg_height:.3f}m at ({x:.3f}, {y:.3f}, {z:.3f})")
        time.sleep(0.3)
    
    def remove_object(self, object_id: str):
        """Remove an object from the planning scene."""
        
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True
        self.planning_scene_pub.publish(scene)
        
        self.get_logger().info(f"Removed '{object_id}' from scene")
        time.sleep(0.3)
    
    def clear_all_objects(self):
        """Remove all spawned objects from the planning scene."""
        # List of known object IDs to clear
        objects_to_clear = [
            'table', 'cylinder_1', 'cylinder_2', 
            'cylinder_bottom', 'cylinder_top',
            'cyl_a', 'cyl_b', 'cyl_c',
            'target_cylinder'
        ]
        
        self.get_logger().info("Clearing planning scene...")
        for obj_id in objects_to_clear:
            obj = CollisionObject()
            obj.header.frame_id = 'base_link'
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.id = obj_id
            obj.operation = CollisionObject.REMOVE
            
            scene = PlanningScene()
            scene.world.collision_objects.append(obj)
            scene.is_diff = True
            self.planning_scene_pub.publish(scene)
        
        time.sleep(0.5)
        self.get_logger().info("Planning scene cleared")
    
    def run_mtc_pick_place(self, object_id: str, radius: float, height: float,
                           pick_x: float, pick_y: float, pick_z: float,
                           place_x: float, place_y: float, place_z: float,
                           spawn_object: bool = False, spawn_table: bool = False):
        """Run the MTC pick-and-place node with given parameters."""
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Running MTC for '{object_id}'")
        self.get_logger().info(f"  Pick:  ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})")
        self.get_logger().info(f"  Place: ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})")
        self.get_logger().info(f"{'='*60}\n")
        
        # Use ros2 launch to ensure robot description is available
        cmd = [
            'ros2', 'launch', 'arm_mtc', 'mtc_run.launch.py',
            f'object_id:={object_id}',
            f'object_radius:={radius}',
            f'object_height:={height}',
            f'pick_x:={pick_x}',
            f'pick_y:={pick_y}',
            f'pick_z:={pick_z}',
            f'place_x:={place_x}',
            f'place_y:={place_y}',
            f'place_z:={place_z}',
            f'spawn_object:={str(spawn_object).lower()}',
            # f'spawn_table:={str(spawn_table).lower()}',
        ]
        
        self.get_logger().info(f"Command: {' '.join(cmd)}")
        
        # Run MTC - node will exit cleanly after task completes
        result = subprocess.run(cmd)
        
        if result.returncode == 0:
            self.get_logger().info(f"✓ Successfully completed pick-place for '{object_id}'")
            return True
        else:
            self.get_logger().error(f"✗ Failed pick-place for '{object_id}' (exit code: {result.returncode})")
            return False


def run_single_cylinder_demo(orchestrator: MTCOrchestrator):
    """Demo: Pick and place a single cylinder."""
    
    orchestrator.get_logger().info("\n" + "="*60)
    orchestrator.get_logger().info("DEMO: Single Cylinder Pick-and-Place")
    orchestrator.get_logger().info("="*60 + "\n")
    
    # Clear any existing objects first
    orchestrator.clear_all_objects()
    
    # Cylinder dimensions
    cylinder_radius = 0.025  # 2.5cm
    cylinder_height = 0.15   # 15cm
    
    # Table dimensions
    table_diameter = 0.25    # 25cm
    table_height = 0.025     # 2.5cm leg
    
    # Target cylinder center height (known to work)
    cylinder_center_z = 1.2
    
    # Table top should be at cylinder_center - half cylinder height
    table_top_z = cylinder_center_z - cylinder_height / 2.0  # 1.2 - 0.075 = 1.125
    
    # Positions (table tops)
    pick_x, pick_y = -0.2, 0.5
    place_x, place_y = -0.2, -0.5
    
    # Spawn round tables at pick and place positions
    orchestrator.spawn_round_table(
        table_id='pick_table',
        x=pick_x, y=pick_y, z=table_top_z,
        diameter=table_diameter, leg_height=table_height
    )
    orchestrator.spawn_round_table(
        table_id='place_table',
        x=place_x, y=place_y, z=table_top_z,
        diameter=table_diameter, leg_height=table_height
    )
    
    # Spawn cylinder on top of pick table (center at cylinder_center_z)
    orchestrator.spawn_cylinder(
        object_id='cylinder_1',
        radius=cylinder_radius,
        height=cylinder_height,
        x=pick_x, y=pick_y, z=cylinder_center_z
    )
    
    # Run MTC
    orchestrator.run_mtc_pick_place(
        object_id='cylinder_1',
        radius=cylinder_radius,
        height=cylinder_height,
        pick_x=pick_x, pick_y=pick_y, pick_z=cylinder_center_z,
        place_x=place_x, place_y=place_y, place_z=cylinder_center_z,
        spawn_object=False,
        spawn_table=False
    )


def run_stacking_demo(orchestrator: MTCOrchestrator):
    """Demo: Stack two cylinders."""
    
    orchestrator.get_logger().info("\n" + "="*60)
    orchestrator.get_logger().info("DEMO: Stacking Two Cylinders")
    orchestrator.get_logger().info("="*60 + "\n")
    
    # Clear any existing objects first
    orchestrator.clear_all_objects()
    
    # Configuration
    cylinder_radius = 0.025  # 2.5cm radius (same as single demo)
    cylinder_height = 0.15   # 15cm height (same as single demo)
    
    # Base positions
    base_z = 1.2
    
    # Stack location (where cylinders will be stacked)
    stack_x = -0.2
    stack_y = -0.5
    
    # Pick positions - use y=0.5 which is known to work, separate on X axis
    pick1_x, pick1_y = -0.1, 0.5   # First cylinder (slightly to one side)
    pick2_x, pick2_y = -0.3, 0.5   # Second cylinder (slightly to other side)
    
    # Spawn table (commented out - may cause issues)
    # orchestrator.spawn_table()
    
    # Spawn first cylinder (will be bottom of stack)
    orchestrator.spawn_cylinder(
        object_id='cylinder_bottom',
        radius=cylinder_radius,
        height=cylinder_height,
        x=pick1_x, y=pick1_y, z=base_z
    )
    
    # Spawn second cylinder (will be on top)
    orchestrator.spawn_cylinder(
        object_id='cylinder_top',
        radius=cylinder_radius,
        height=cylinder_height,
        x=pick2_x, y=pick2_y, z=base_z
    )
    
    time.sleep(1.0)  # Let scene settle
    
    # Step 1: Move bottom cylinder to stack position
    orchestrator.get_logger().info("\n--- Step 1: Place bottom cylinder ---")
    success = orchestrator.run_mtc_pick_place(
        object_id='cylinder_bottom',
        radius=cylinder_radius,
        height=cylinder_height,
        pick_x=pick1_x, pick_y=pick1_y, pick_z=base_z,
        place_x=stack_x, place_y=stack_y, place_z=base_z,
        spawn_object=False,
        spawn_table=False
    )
    
    if not success:
        orchestrator.get_logger().error("Failed to place bottom cylinder, aborting")
        return False
    
    time.sleep(2.0)  # Give more time for scene to update
    
    # Step 2: Stack top cylinder on bottom
    # Stack height = base_z + bottom_cylinder_height
    stack_z = base_z + cylinder_height
    
    orchestrator.get_logger().info("\n--- Step 2: Stack top cylinder ---")
    success2 = orchestrator.run_mtc_pick_place(
        object_id='cylinder_top',
        radius=cylinder_radius,
        height=cylinder_height,
        pick_x=pick2_x, pick_y=pick2_y, pick_z=base_z,
        place_x=stack_x, place_y=stack_y, place_z=stack_z,
        spawn_object=False,
        spawn_table=False
    )
    
    if success2:
        orchestrator.get_logger().info("\n" + "="*60)
        orchestrator.get_logger().info("STACKING COMPLETE!")
        orchestrator.get_logger().info("="*60 + "\n")
    else:
        orchestrator.get_logger().error("Failed to stack top cylinder")
    
    return success2


def run_sequence_demo(orchestrator: MTCOrchestrator):
    """Demo: Move multiple cylinders one after another."""
    
    orchestrator.get_logger().info("\n" + "="*60)
    orchestrator.get_logger().info("DEMO: Sequential Pick-and-Place")
    orchestrator.get_logger().info("="*60 + "\n")
    
    # Clear any existing objects first
    orchestrator.clear_all_objects()
    
    # Define objects to manipulate
    objects = [
        {
            'id': 'cyl_a',
            'radius': 0.002,  # 2mm
            'height': 0.08,   # 8cm
            'pick': (-0.2, 0.4, 1.2),
            'place': (-0.3, -0.4, 1.2),
        },
        {
            'id': 'cyl_b',
            'radius': 0.0025,  # 2.5mm
            'height': 0.10,    # 10cm
            'pick': (-0.2, 0.5, 1.2),
            'place': (-0.2, -0.5, 1.2),
        },
        # {
        #     'id': 'cyl_c',
        #     'radius': 0.003,  # 3mm
        #     'height': 0.12,   # 12cm
        #     'pick': (-0.2, 0.6, 1.2),
        #     'place': (-0.1, -0.6, 1.2),
        # },
    ]
    
    # Spawn table
    orchestrator.spawn_table()
    
    # Spawn all objects
    for obj in objects:
        orchestrator.spawn_cylinder(
            object_id=obj['id'],
            radius=obj['radius'],
            height=obj['height'],
            x=obj['pick'][0], y=obj['pick'][1], z=obj['pick'][2]
        )
    
    time.sleep(1.0)
    
    # Pick and place each object
    for i, obj in enumerate(objects):
        orchestrator.get_logger().info(f"\n--- Object {i+1}/{len(objects)}: {obj['id']} ---")
        
        success = orchestrator.run_mtc_pick_place(
            object_id=obj['id'],
            radius=obj['radius'],
            height=obj['height'],
            pick_x=obj['pick'][0], pick_y=obj['pick'][1], pick_z=obj['pick'][2],
            place_x=obj['place'][0], place_y=obj['place'][1], place_z=obj['place'][2],
            spawn_object=False,
            spawn_table=False
        )
        
        if not success:
            orchestrator.get_logger().warn(f"Failed on {obj['id']}, continuing...")
        
        time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description='MTC Orchestrator')
    parser.add_argument('--scene', type=str, default='single',
                        choices=['single', 'stack', 'sequence'],
                        help='Demo scene to run')
    
    # Parse known args to handle ROS args
    args, unknown = parser.parse_known_args()
    
    rclpy.init(args=unknown if unknown else None)
    
    orchestrator = MTCOrchestrator()
    
    try:
        if args.scene == 'single':
            run_single_cylinder_demo(orchestrator)
        elif args.scene == 'stack':
            run_stacking_demo(orchestrator)
        elif args.scene == 'sequence':
            run_sequence_demo(orchestrator)
        
        orchestrator.get_logger().info("\n" + "="*60)
        orchestrator.get_logger().info("ORCHESTRATION COMPLETE")
        orchestrator.get_logger().info("="*60 + "\n")
        
    except KeyboardInterrupt:
        orchestrator.get_logger().info("Interrupted by user")
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
