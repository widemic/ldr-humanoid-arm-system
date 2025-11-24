#!/usr/bin/env python3
"""
Pick and Place Task - Using MoveGroup Action Client

This version uses the standard MoveGroup action interface instead of MoveItPy,
which is more reliable when move_group is already running.

Usage:
    ros2 run arm_perception pick_and_place_action.py

Prerequisites:
    - Gazebo: ros2 launch arm_control sim.launch.py
    - MoveIt: ros2 launch arm_moveit_config demo.launch.py
    - Test env: ros2 launch arm_perception test_environment.launch.py

Author: LDR Humanoid Arm System
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    BoundingVolume,
    JointConstraint,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from std_msgs.msg import Header
import time


class PickAndPlaceAction(Node):
    """Pick and place using MoveGroup action client"""

    def __init__(self):
        super().__init__('pick_and_place_action')

        # Configuration - matches test_environment_publisher.py EXACTLY
        self.FRAME_ID = 'base_link'
        self.GROUP_NAME = 'arm'
        self.END_EFFECTOR_LINK = 'left_palm'

        # Source table position (MUST match test_environment_publisher.py)
        self.TABLE_X = -0.3
        self.TABLE_Y = 0.35
        self.TABLE_Z = 0.9
        self.TABLE_THICKNESS = 0.02
        self.CYLINDER_HEIGHT = 0.15

        # Destination table position (MUST match test_environment_publisher.py)
        self.DEST_TABLE_X = -0.7
        self.DEST_TABLE_Y = 0.0
        self.DEST_TABLE_Z = 0.9

        # Calculate cylinder position on source table
        table_top_z = self.TABLE_Z + (self.TABLE_THICKNESS / 2.0)
        self.cylinder_z = table_top_z + (self.CYLINDER_HEIGHT / 2.0)
        self.cylinder_x = self.TABLE_X
        self.cylinder_y = self.TABLE_Y

        # Calculate destination position on dest table
        dest_table_top_z = self.DEST_TABLE_Z + (self.TABLE_THICKNESS / 2.0)
        self.dest_z = dest_table_top_z + (self.CYLINDER_HEIGHT / 2.0)
        self.dest_x = self.DEST_TABLE_X
        self.dest_y = self.DEST_TABLE_Y

        # Motion parameters
        # Gripper geometry: fingers extend ~5cm in Y direction from palm
        # NOTE: palm_fixture has rotation rpy="-0.77494 -0.017568 0.017203" (~-44° pitch)
        #       This means the gripper naturally points at an angle, not straight down.
        #       For now, we let MoveIt find any valid orientation (unconstrained).
        #       If vertical grasping is needed, add orientation constraints to pose goals.
        self.GRASP_OFFSET_Z = 0.0     # Aim the palm at the cylinder center
        self.PLACE_OFFSET_Z = 0.03    # Leave a 3cm air gap when lowering to place
        self.APPROACH_HEIGHT = 0.10   # Approach from 10cm above grasp position
        self.LIFT_HEIGHT = 0.15       # Lift height after grasp
        self.TRANSFER_HEIGHT = 1.15   # Absolute height for safe transfer between tables

        # Action client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # Action client for Gripper
        self._gripper_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')

        # Wait for action servers
        self.get_logger().info('Waiting for move_group action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✓ Connected to move_group')

        self.get_logger().info('Waiting for gripper action server...')
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('⚠ Gripper server not available, gripper commands will be simulated')
            self.gripper_available = False
        else:
            self.get_logger().info('✓ Connected to gripper')
            self.gripper_available = True

        # Publishers for attach/detach
        self.attached_obj_pub = self.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10)
        self.collision_obj_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)

        # Precompute an allowed collision matrix so the gripper can touch the cylinder and
        # the cylinder can touch the tables without MoveIt flagging start-state collisions.
        self.allowed_collision_matrix = self.build_allowed_collision_matrix()

        self.get_logger().info(f'Cylinder at: ({self.cylinder_x:.3f}, {self.cylinder_y:.3f}, {self.cylinder_z:.3f})')
        self.get_logger().info(f'Destination at: ({self.dest_x:.3f}, {self.dest_y:.3f}, {self.dest_z:.3f})')

    def create_pose_goal(self, x, y, z, frame_id='base_link', use_orientation=False):
        """Create a pose goal for MoveGroup

        Args:
            x, y, z: Target position
            frame_id: Reference frame
            use_orientation: If True, constrains gripper to point downward (better for grasping)
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z

        if use_orientation:
            # Gripper pointing downward (Z-axis down) for vertical grasping
            # This compensates for the palm_fixture rotation
            # Quaternion for 90° rotation around Y axis (pointing down)
            import math
            angle = math.pi / 2  # 90 degrees
            pose_stamped.pose.orientation.w = math.cos(angle / 2)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = math.sin(angle / 2)
            pose_stamped.pose.orientation.z = 0.0
        else:
            # Identity orientation (no specific orientation required)
            # The planner will find any valid orientation to reach this position
            pose_stamped.pose.orientation.w = 1.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0

        return pose_stamped

    def build_allowed_collision_matrix(self):
        """Allow the gripper and tables to be in contact with the cylinder during pick/place"""
        names = [
            'test_cylinder',
            'left_palm',
            'left_palm_left_finger_link',
            'left_palm_right_finger_link',
            'test_table',
            'destination_table',
            # Pairs reported by CheckStartStateCollision (allow to bypass false positives)
            'base_link',
            'left_shoulder_pitch_rs04_actuator',
            'left_elbow',
            'left_elbow_rs03_actuator',
            'left_elbow_rs03_flange',
            'left_wrist_actuator',
            'left_shoulder_yaw',
            'left_hand',
            'left_hand_rs02_actuator',
            'left_hand_rs02_flange',
            'left_wrist',
            'left_shoulder_pitch',
            'left_shoulder_pitch_rs04_flange',
            'left_shoulder_roll_rs04_flange',
            'left_shoulder_roll',
            'left_shoulder_roll_rs04_actuator',
            'left_shoulder_yaw_rs03_actuator',
            'left_shoulder_yaw_flange',
            'left_shoulder_yaw_rs03_flange',
            'left_wrist_rs02_flange',
        ]

        acm = AllowedCollisionMatrix()
        acm.entry_names = names

        # Initialize symmetric matrix of False
        for _ in names:
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * len(names)
            acm.entry_values.append(entry)

        def allow(a, b):
            if a not in names or b not in names:
                # Defensive: skip pairs that are not present to avoid ValueError
                return
            i = names.index(a)
            j = names.index(b)
            acm.entry_values[i].enabled[j] = True
            acm.entry_values[j].enabled[i] = True

        # Allow gripper links and tables to touch the cylinder
        allow('test_cylinder', 'left_palm')
        allow('test_cylinder', 'left_palm_left_finger_link')
        allow('test_cylinder', 'left_palm_right_finger_link')
        allow('test_cylinder', 'test_table')
        allow('test_cylinder', 'destination_table')
        # Allow palm to make light contact with tables near place
        allow('left_palm', 'test_table')
        allow('left_palm', 'destination_table')

        # Allow all reported start-state self-collision pairs (planner then ignores them)
        reported_pairs = [
            ('base_link', 'left_shoulder_pitch_rs04_actuator'),
            ('left_elbow', 'left_elbow_rs03_actuator'),
            ('left_elbow', 'left_elbow_rs03_flange'),
            ('left_elbow', 'left_wrist_actuator'),
            ('left_elbow_rs03_actuator', 'left_shoulder_yaw'),
            ('left_hand', 'left_hand_rs02_actuator'),
            ('left_hand', 'left_hand_rs02_flange'),
            ('left_hand', 'left_palm'),
            ('left_hand_rs02_actuator', 'left_hand_rs02_flange'),
            ('left_hand_rs02_actuator', 'left_wrist'),
            ('left_palm', 'left_palm_left_finger_link'),
            ('left_palm', 'left_palm_right_finger_link'),
            ('left_shoulder_pitch', 'left_shoulder_pitch_rs04_actuator'),
            ('left_shoulder_pitch', 'left_shoulder_pitch_rs04_flange'),
            ('left_shoulder_pitch', 'left_shoulder_roll_rs04_flange'),
            ('left_shoulder_pitch_rs04_actuator', 'left_shoulder_pitch_rs04_flange'),
            ('left_shoulder_roll', 'left_shoulder_roll_rs04_actuator'),
            ('left_shoulder_roll', 'left_shoulder_yaw_rs03_actuator'),
            ('left_shoulder_yaw', 'left_shoulder_yaw_flange'),
            ('left_shoulder_yaw_flange', 'left_shoulder_yaw_rs03_flange'),
            ('left_wrist', 'left_wrist_rs02_flange'),
            ('left_wrist_actuator', 'left_wrist_rs02_flange'),
        ]
        for a, b in reported_pairs:
            allow(a, b)

        return acm

    def send_goal_and_wait(self, pose_goal, description="Move"):
        """Send goal to MoveGroup and wait for result"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.GROUP_NAME
        goal_msg.request.num_planning_attempts = 20  # Increased from 10
        goal_msg.request.allowed_planning_time = 15.0  # Increased from 5s
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # Define workspace
        goal_msg.request.workspace_parameters.header.frame_id = self.FRAME_ID
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.min_corner.z = 0.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.0

        # Set pose goal with RELAXED constraints
        goal_msg.request.goal_constraints.append(Constraints())
        goal_msg.request.goal_constraints[0].name = "pose_goal"

        # Position constraint with larger tolerance
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = self.END_EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        # Bounding volume (larger sphere for easier planning)
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 5cm tolerance (increased from 1cm)
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(pose_goal.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        goal_msg.request.goal_constraints[0].position_constraints.append(pos_constraint)

        # NO orientation constraint - let the planner find any valid orientation
        # This dramatically improves reachability at the cost of less control over approach angle
        # For pick and place, position is more critical than exact orientation
        # (Orientation constraint removed to allow IK solver more freedom)

        # CRITICAL: Set to PLAN AND EXECUTE (not just plan)
        goal_msg.planning_options.plan_only = False  # Plan AND execute
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.planning_scene_diff.allowed_collision_matrix = self.allowed_collision_matrix

        self.get_logger().info(f'{description}...')
        self.get_logger().info(f'  Target: ({pose_goal.pose.position.x:.3f}, {pose_goal.pose.position.y:.3f}, {pose_goal.pose.position.z:.3f})')

        # Send goal with feedback callback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'✗ Goal rejected: {description}')
            return False

        self.get_logger().info('  Planning and executing...')

        # Wait for result (this includes execution time)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f'✓ {description} succeeded')
            return True
        else:
            self.get_logger().error(f'✗ {description} failed (error code: {result.error_code.val})')
            return False

    def feedback_callback(self, feedback_msg):
        """Receive feedback during planning/execution"""
        feedback = feedback_msg.feedback
        if feedback.state:
            self.get_logger().info(f'  Status: {feedback.state}', throttle_duration_sec=2.0)

    def control_gripper(self, position, description="Gripper", max_effort=10.0):
        """Control gripper

        Args:
            position: Target position (meters)
                -0.033m = fully open
                 0.0m = fully closed
            description: Description for logging
            max_effort: Maximum effort (N). Lower values for gentle grasping.
        """
        if not self.gripper_available:
            self.get_logger().info(f'{description} [SIMULATED]')
            time.sleep(0.5)
            return True

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'{description}...')
        self.get_logger().info(f'  Position: {position:.4f}m, Max effort: {max_effort:.1f}N')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'✗ {description} rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

        result = result_future.result()
        if result:
            self.get_logger().info(f'✓ {description} done (final position: {result.result.position:.4f}m)')
        else:
            self.get_logger().info(f'✓ {description} done')
        return True

    def open_gripper(self):
        """Open gripper fully (prismatic joint: -0.033 = open)"""
        return self.control_gripper(-0.030, "Open gripper")

    def close_gripper(self):
        """Close gripper to grasp cylinder (diameter 6cm)

        Gripper joint range: [-0.033m (open), 0.0m (closed)]
        For a 6cm diameter cylinder, we need ~3cm distance from center to finger.
        With stall detection enabled, we command slightly past the object
        and let the controller stop when it touches the cylinder.
        """
        # Command position that would grasp object (controller will stall at contact)
        # Cylinder radius = 3cm, so we aim for ~-0.015m to -0.010m
        # Using -0.012m to ensure good contact without going through
        return self.control_gripper(-0.012, "Close gripper to grasp")

    def allow_gripper_cylinder_collision(self):
        """Nothing to send: ACM is already set on each goal"""
        self.get_logger().info('✓ Collision matrix already relaxes cylinder contact')

    def attach_cylinder(self):
        """Attach cylinder to gripper in planning scene"""
        msg = AttachedCollisionObject()
        msg.link_name = self.END_EFFECTOR_LINK
        msg.object.header.frame_id = self.END_EFFECTOR_LINK
        msg.object.header.stamp = self.get_clock().now().to_msg()
        msg.object.id = 'test_cylinder'
        msg.object.operation = CollisionObject.ADD

        # Cylinder shape - match actual cylinder dimensions
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [self.CYLINDER_HEIGHT, 0.03]  # height=15cm, radius=3cm

        # Position relative to end effector
        # The palm_fixture is rotated ~-44° from the hand frame
        # When grasping, the cylinder is held between the fingers
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.06  # Offset in Y direction (along finger axis)
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        msg.object.primitives.append(cyl)
        msg.object.primitive_poses.append(pose)
        msg.touch_links = ['left_palm', 'left_palm_left_finger_link', 'left_palm_right_finger_link']

        self.attached_obj_pub.publish(msg)
        self.get_logger().info('✓ Cylinder attached to gripper')
        time.sleep(0.3)

    def detach_cylinder(self, place_x, place_y, place_z):
        """Detach cylinder and place it at specified position"""
        # First detach
        detach_msg = AttachedCollisionObject()
        detach_msg.link_name = self.END_EFFECTOR_LINK
        detach_msg.object.id = 'test_cylinder'
        detach_msg.object.operation = CollisionObject.REMOVE
        self.attached_obj_pub.publish(detach_msg)
        time.sleep(0.2)

        # Then add cylinder at new position
        obj = CollisionObject()
        obj.header.frame_id = self.FRAME_ID
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = 'test_cylinder'
        obj.operation = CollisionObject.ADD

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [self.CYLINDER_HEIGHT, 0.03]

        pose = Pose()
        pose.position.x = place_x
        pose.position.y = place_y
        pose.position.z = place_z
        pose.orientation.w = 1.0

        obj.primitives.append(cyl)
        obj.primitive_poses.append(pose)
        self.collision_obj_pub.publish(obj)

        self.get_logger().info(f'✓ Cylinder placed at ({place_x:.2f}, {place_y:.2f}, {place_z:.2f})')
        time.sleep(0.3)

    def execute_pick_and_place(self):
        """Execute simple pick and place: open gripper -> position -> close -> move -> open"""
        try:
            # Calculate positions
            # Grasp: palm aligned to cylinder center so fingers wrap around middle
            grasp_z = self.cylinder_z + self.GRASP_OFFSET_Z
            approach_z = grasp_z + self.APPROACH_HEIGHT

            # Destination: leave a small air gap over the table to avoid contact before detach
            dest_grasp_z = self.dest_z + self.PLACE_OFFSET_Z

            self.get_logger().info('=' * 60)
            self.get_logger().info('PICK AND PLACE')
            self.get_logger().info(f'  Cylinder center: Z={self.cylinder_z:.3f}m (height={self.CYLINDER_HEIGHT}m)')
            self.get_logger().info(f'  Grasp position: Z={grasp_z:.3f}m (palm {self.GRASP_OFFSET_Z}m above center)')
            self.get_logger().info(f'  Approach: Z={approach_z:.3f}m')
            self.get_logger().info(f'  Transfer height: Z={self.TRANSFER_HEIGHT:.3f}m')
            self.get_logger().info(f'  Destination: ({self.dest_x:.2f}, {self.dest_y:.2f}, {self.dest_z:.2f})')
            self.get_logger().info('=' * 60)

            # Step 1: Open gripper
            self.get_logger().info('Step 1: Opening gripper...')
            self.open_gripper()
            time.sleep(1.0)

            # Step 2: Move above cylinder (approach)
            self.get_logger().info(f'Step 2: Approach above cylinder (Z={approach_z:.3f})...')
            pose = self.create_pose_goal(self.cylinder_x, self.cylinder_y, approach_z)
            if not self.send_goal_and_wait(pose, "Approach"):
                return False
            time.sleep(0.5)

            # Allow gripper to touch cylinder (for grasping)
            self.get_logger().info('Step 2b: Relax collisions on cylinder for grasp (ACM update only)...')
            self.allow_gripper_cylinder_collision()

            # Step 3: Lower to grasp position
            self.get_logger().info(f'Step 3: Lower to grasp (Z={grasp_z:.3f})...')
            pose = self.create_pose_goal(self.cylinder_x, self.cylinder_y, grasp_z)
            if not self.send_goal_and_wait(pose, "Lower to grasp"):
                return False
            time.sleep(0.5)

            # Step 4: Close gripper to grasp
            self.get_logger().info('Step 4: Closing gripper...')
            self.close_gripper()
            time.sleep(1.0)

            # Attach cylinder to gripper
            self.attach_cylinder()

            # Step 5: Lift to transfer height (safe height above both tables)
            self.get_logger().info(f'Step 5: Lifting to transfer height (Z={self.TRANSFER_HEIGHT:.3f})...')
            pose = self.create_pose_goal(self.cylinder_x, self.cylinder_y, self.TRANSFER_HEIGHT)
            if not self.send_goal_and_wait(pose, "Lift to transfer height"):
                return False
            time.sleep(0.5)

            # Step 6: Move to destination at transfer height
            self.get_logger().info(f'Step 6: Moving to destination ({self.dest_x:.2f}, {self.dest_y:.2f}) at Z={self.TRANSFER_HEIGHT:.3f}...')
            pose = self.create_pose_goal(self.dest_x, self.dest_y, self.TRANSFER_HEIGHT)
            if not self.send_goal_and_wait(pose, "Move to dest"):
                return False
            time.sleep(0.5)

            # Step 7: Lower to place with fallback heights
            place_heights = [dest_grasp_z, dest_grasp_z + 0.03]
            placed = False
            for idx, z_target in enumerate(place_heights, start=1):
                self.get_logger().info(f'Step 7: Lower to place attempt {idx} (Z={z_target:.3f})...')
                pose = self.create_pose_goal(self.dest_x, self.dest_y, z_target)
                if self.send_goal_and_wait(pose, "Lower to place"):
                    placed = True
                    break
                else:
                    self.get_logger().warn(f'Place attempt {idx} failed, trying higher Z...')
            if not placed:
                return False
            time.sleep(0.5)

            # Step 8: Open gripper to release
            self.get_logger().info('Step 8: Opening gripper to release...')
            self.open_gripper()
            time.sleep(1.0)

            # Detach cylinder at new position
            self.detach_cylinder(self.dest_x, self.dest_y, self.dest_z)

            # Step 9: Retreat to safe height
            self.get_logger().info(f'Step 9: Retreating to safe height (Z={self.TRANSFER_HEIGHT:.3f})...')
            pose = self.create_pose_goal(self.dest_x, self.dest_y, self.TRANSFER_HEIGHT)
            if not self.send_goal_and_wait(pose, "Retreat"):
                return False

            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ PICK AND PLACE COMPLETED!')
            self.get_logger().info('=' * 60)

            return True

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            import traceback
            traceback.print_exc()
            return False


def main(args=None):
    rclpy.init(args=args)

    task = PickAndPlaceAction()

    # Wait for initialization
    time.sleep(2.0)

    try:
        success = task.execute_pick_and_place()

        if success:
            task.get_logger().info('Task completed!')
        else:
            task.get_logger().error('Task failed!')

    except KeyboardInterrupt:
        task.get_logger().info('Interrupted by user')
    finally:
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
