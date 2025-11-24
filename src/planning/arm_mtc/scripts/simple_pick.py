#!/usr/bin/env python3
"""
Simple pick demo - no MTC complexity, just basic MoveIt commands
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time

class SimplePickDemo(Node):
    def __init__(self):
        super().__init__('simple_pick_demo')

        self.get_logger().info("Initializing MoveIt...")
        self.moveit = MoveItPy(node_name="simple_pick_demo")

        self.arm = self.moveit.get_planning_component("arm")
        self.hand = self.moveit.get_planning_component("hand")

        self.get_logger().info("MoveIt initialized successfully!")

    def run_demo(self):
        """Run a simple pick sequence"""

        # Step 1: Move to home position
        self.get_logger().info("Step 1: Moving to home position...")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(2)
        else:
            self.get_logger().error("Planning to home failed!")
            return False

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        self.hand.set_start_state_to_current_state()
        self.hand.set_goal_state(configuration_name="open")

        plan_result = self.hand.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(1)
        else:
            self.get_logger().error("Planning gripper open failed!")
            return False

        # Step 3: Move to pre-grasp position (above object)
        self.get_logger().info("Step 3: Moving to pre-grasp position...")
        self.arm.set_start_state_to_current_state()

        # Target position: x=0.0, y=0.4, z=1.25 (above the object at z=1.15)
        target_pose = {
            "position": {"x": 0.0, "y": 0.4, "z": 1.25},
            "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}  # Gripper pointing down
        }

        self.arm.set_goal_state(pose_stamped_msg=self.create_pose_stamped(target_pose), pose_link="left_hand")

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(2)
        else:
            self.get_logger().error("Planning to pre-grasp failed!")
            return False

        # Step 4: Move down to grasp position
        self.get_logger().info("Step 4: Moving down to grasp...")
        self.arm.set_start_state_to_current_state()

        target_pose["position"]["z"] = 1.15  # Move down to object height
        self.arm.set_goal_state(pose_stamped_msg=self.create_pose_stamped(target_pose), pose_link="left_hand")

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(2)
        else:
            self.get_logger().error("Planning to grasp failed!")
            return False

        # Step 5: Close gripper
        self.get_logger().info("Step 5: Closing gripper to grasp...")
        self.hand.set_start_state_to_current_state()
        self.hand.set_goal_state(configuration_name="close")

        plan_result = self.hand.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(1)
        else:
            self.get_logger().error("Planning gripper close failed!")
            return False

        # Step 6: Lift object
        self.get_logger().info("Step 6: Lifting object...")
        self.arm.set_start_state_to_current_state()

        target_pose["position"]["z"] = 1.30  # Lift up
        self.arm.set_goal_state(pose_stamped_msg=self.create_pose_stamped(target_pose), pose_link="left_hand")

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(2)
        else:
            self.get_logger().error("Planning lift failed!")
            return False

        # Step 7: Return to home
        self.get_logger().info("Step 7: Returning to home...")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")

        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            time.sleep(2)
        else:
            self.get_logger().error("Planning to home failed!")
            return False

        self.get_logger().info("=== DEMO COMPLETE ===")
        return True

    def create_pose_stamped(self, pose_dict):
        """Helper to create PoseStamped message"""
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = pose_dict["position"]["x"]
        pose.pose.position.y = pose_dict["position"]["y"]
        pose.pose.position.z = pose_dict["position"]["z"]
        pose.pose.orientation.x = pose_dict["orientation"]["x"]
        pose.pose.orientation.y = pose_dict["orientation"]["y"]
        pose.pose.orientation.z = pose_dict["orientation"]["z"]
        pose.pose.orientation.w = pose_dict["orientation"]["w"]

        return pose


def main(args=None):
    rclpy.init(args=args)

    demo = SimplePickDemo()

    try:
        demo.get_logger().info("Starting simple pick demo in 3 seconds...")
        time.sleep(3)

        success = demo.run_demo()

        if success:
            demo.get_logger().info("Demo completed successfully!")
        else:
            demo.get_logger().error("Demo failed!")

        # Keep node alive for a bit
        demo.get_logger().info("Keeping node alive. Press Ctrl+C to exit.")
        rclpy.spin(demo)

    except KeyboardInterrupt:
        demo.get_logger().info("Demo interrupted by user")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
