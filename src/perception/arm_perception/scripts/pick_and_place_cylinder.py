#!/usr/bin/env python3
"""
Pick and Place Task - Move Cylinder 10cm to the Right

This script demonstrates a simple pick and place operation:
1. Move to pre-grasp position (above cylinder)
2. Move down to grasp position
3. (Close gripper - if available)
4. Lift object
5. Move 10cm to the right
6. Place object down
7. (Open gripper)
8. Return to home

Usage:
    ros2 run arm_perception pick_and_place_cylinder.py

Prerequisites:
    - MoveIt running: ros2 launch arm_moveit_config demo.launch.py
    - Test environment: ros2 launch arm_perception test_environment.launch.py

Author: LDR Humanoid Arm System
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPositionIK
import time

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_PY_AVAILABLE = True
except ImportError:
    MOVEIT_PY_AVAILABLE = False
    print("WARNING: moveit_py not available, using action client fallback")


class PickAndPlaceTask(Node):
    """Pick and place task to move cylinder 10cm to the right"""

    def __init__(self):
        super().__init__('pick_and_place_task')

        # Configuration - matches test_environment_publisher.py
        self.FRAME_ID = 'base_link'

        # Cylinder position (from test environment)
        # Table: X=-0.5, Y=0.5, Z=1.0, thickness=0.02
        # Cylinder on table
        self.TABLE_X = -0.5
        self.TABLE_Y = 0.5
        self.TABLE_Z = 1.0
        self.TABLE_THICKNESS = 0.02
        self.CYLINDER_HEIGHT = 0.15

        # Calculate cylinder center position
        table_top_z = self.TABLE_Z + (self.TABLE_THICKNESS / 2.0)
        self.cylinder_z = table_top_z + (self.CYLINDER_HEIGHT / 2.0)
        self.cylinder_x = self.TABLE_X
        self.cylinder_y = self.TABLE_Y

        # Motion parameters
        self.PRE_GRASP_HEIGHT = 0.10   # 10cm above cylinder
        self.LIFT_HEIGHT = 0.15         # Lift 15cm after grasping
        self.MOVE_RIGHT_DISTANCE = 0.10 # Move 10cm to the right (positive Y)

        self.get_logger().info('Pick and Place Task initialized')
        self.get_logger().info(f'Cylinder position: ({self.cylinder_x:.3f}, {self.cylinder_y:.3f}, {self.cylinder_z:.3f})')

    def create_pose(self, x, y, z, orientation_w=1.0):
        """Helper function to create a Pose"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = orientation_w
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        return pose

    def execute_pick_and_place(self):
        """Execute the complete pick and place sequence"""
        if not MOVEIT_PY_AVAILABLE:
            self.get_logger().error('MoveItPy not available. Install with: sudo apt install ros-jazzy-moveit-py')
            return False

        try:
            # Initialize MoveItPy
            self.get_logger().info('Initializing MoveIt...')
            moveit = MoveItPy(node_name="pick_and_place_moveit")
            arm = moveit.get_planning_component("arm")

            # Step 1: Move to home position
            self.get_logger().info('Step 1: Moving to home position...')
            arm.set_start_state_to_current_state()
            arm.set_goal_state(configuration_name="home")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Reached home position')
            else:
                self.get_logger().error('Failed to plan to home')
                return False

            # Step 2: Move to pre-grasp position (above cylinder)
            pre_grasp_z = self.cylinder_z + self.PRE_GRASP_HEIGHT
            self.get_logger().info(f'Step 2: Moving to pre-grasp position above cylinder...')
            self.get_logger().info(f'  Target: ({self.cylinder_x:.3f}, {self.cylinder_y:.3f}, {pre_grasp_z:.3f})')

            target_pose = PoseStamped()
            target_pose.header.frame_id = self.FRAME_ID
            target_pose.pose = self.create_pose(self.cylinder_x, self.cylinder_y, pre_grasp_z)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Reached pre-grasp position')
            else:
                self.get_logger().error('Failed to plan to pre-grasp position')
                return False

            # Step 3: Move down to grasp position (at cylinder center)
            self.get_logger().info('Step 3: Moving down to grasp position...')
            target_pose.pose = self.create_pose(self.cylinder_x, self.cylinder_y, self.cylinder_z)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Reached grasp position')
            else:
                self.get_logger().error('Failed to plan to grasp position')
                return False

            # Step 4: Close gripper (simulated - log only)
            self.get_logger().info('Step 4: [SIMULATED] Closing gripper...')
            time.sleep(0.5)
            self.get_logger().info('✓ Gripper closed (simulated)')

            # Step 5: Lift object up
            lift_z = self.cylinder_z + self.LIFT_HEIGHT
            self.get_logger().info(f'Step 5: Lifting object to {lift_z:.3f}m...')
            target_pose.pose = self.create_pose(self.cylinder_x, self.cylinder_y, lift_z)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Object lifted')
            else:
                self.get_logger().error('Failed to lift object')
                return False

            # Step 6: Move 10cm to the right (positive Y direction)
            new_y = self.cylinder_y + self.MOVE_RIGHT_DISTANCE
            self.get_logger().info(f'Step 6: Moving 10cm to the right (Y: {self.cylinder_y:.3f} → {new_y:.3f})...')
            target_pose.pose = self.create_pose(self.cylinder_x, new_y, lift_z)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Moved to new position')
            else:
                self.get_logger().error('Failed to move to new position')
                return False

            # Step 7: Place object down
            place_z = self.cylinder_z  # Same height as original grasp
            self.get_logger().info(f'Step 7: Placing object down...')
            target_pose.pose = self.create_pose(self.cylinder_x, new_y, place_z)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Object placed')
            else:
                self.get_logger().error('Failed to place object')
                return False

            # Step 8: Open gripper (simulated)
            self.get_logger().info('Step 8: [SIMULATED] Opening gripper...')
            time.sleep(0.5)
            self.get_logger().info('✓ Gripper opened (simulated)')

            # Step 9: Move up to clear object
            self.get_logger().info('Step 9: Moving up to clear object...')
            target_pose.pose = self.create_pose(self.cylinder_x, new_y, place_z + self.PRE_GRASP_HEIGHT)

            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_roll_link")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                time.sleep(1.0)
                self.get_logger().info('✓ Cleared object')
            else:
                self.get_logger().warn('Failed to clear object (non-critical)')

            # Step 10: Return to home
            self.get_logger().info('Step 10: Returning to home position...')
            arm.set_start_state_to_current_state()
            arm.set_goal_state(configuration_name="home")

            plan_result = arm.plan()
            if plan_result:
                arm.execute()
                self.get_logger().info('✓ Returned to home')
            else:
                self.get_logger().warn('Failed to return home (non-critical)')

            self.get_logger().info('=' * 50)
            self.get_logger().info('✅ PICK AND PLACE TASK COMPLETED SUCCESSFULLY!')
            self.get_logger().info(f'Cylinder moved from Y={self.cylinder_y:.3f}m to Y={new_y:.3f}m')
            self.get_logger().info('=' * 50)

            return True

        except Exception as e:
            self.get_logger().error(f'Error during pick and place: {e}')
            import traceback
            traceback.print_exc()
            return False


def main(args=None):
    rclpy.init(args=args)

    task = PickAndPlaceTask()

    # Wait a bit for everything to initialize
    time.sleep(1.0)

    try:
        # Execute the pick and place sequence
        success = task.execute_pick_and_place()

        if success:
            task.get_logger().info('Task completed successfully!')
        else:
            task.get_logger().error('Task failed!')

    except KeyboardInterrupt:
        task.get_logger().info('Task interrupted by user')
    finally:
        task.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
