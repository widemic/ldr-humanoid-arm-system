#!/usr/bin/env python3
"""
MoveIt Task Constructor Pick and Place Demo for 5-DOF Humanoid Arm

This script demonstrates how to use MTC to create complex manipulation tasks
with the 5-DOF humanoid arm.
"""

import rclpy
from rclpy.node import Node
from moveit.task_constructor import core, stages
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from shape_msgs.msg import SolidPrimitive
import sys


class PickPlaceTask(Node):
    def __init__(self):
        super().__init__('pick_place_demo')

        # Task setup
        self.task = core.Task()
        self.task.name = "pick_place_task"

        # Planning group (defined in arm_moveit_config SRDF)
        self.arm_group = "arm"

        # Frame IDs
        self.world_frame = "world"
        self.hand_frame = "wrist_roll_link"  # End-effector link

        self.get_logger().info("Initializing Pick and Place Task...")

    def setup_task(self):
        """Configure the MTC task pipeline"""

        # 1. Current State - Start from current robot state
        current_state = stages.CurrentState("current state")
        self.task.add(current_state)

        # 2. Open Hand (if gripper exists - placeholder for future)
        # open_hand = stages.MoveTo("open hand", self.hand_group)
        # open_hand.setGoal("open")
        # self.task.add(open_hand)

        # 3. Move to Pre-Grasp Pose
        move_to_pick = stages.Connect(
            "move to pick",
            stages.Connect.GroupPlannerVector([
                (self.arm_group, "RRTConnect")
            ])
        )
        move_to_pick.setTimeout(5.0)
        move_to_pick.properties.configureInitFrom(core.Stage.PARENT)
        self.task.add(move_to_pick)

        # 4. Approach Object
        approach = stages.MoveRelative("approach object", self.arm_group)
        approach.properties.configureInitFrom(core.Stage.PARENT)
        approach.setMinMaxDistance(0.05, 0.15)
        approach.setIKFrame(self.hand_frame)

        # Set approach direction (downward in z)
        approach.setDirection(Vector3(x=0.0, y=0.0, z=-1.0))
        self.task.add(approach)

        # 5. Generate Grasp Pose (placeholder - would generate multiple grasp poses)
        # For now, we use a fixed pose
        # grasp_generator = stages.GenerateGraspPose("generate grasp pose")
        # self.task.add(grasp_generator)

        # 6. Allow Collision (hand-object)
        # allow_collision = stages.ModifyPlanningScene("allow collision")
        # self.task.add(allow_collision)

        # 7. Close Hand (if gripper exists - placeholder)
        # close_hand = stages.MoveTo("close hand", self.hand_group)
        # close_hand.setGoal("closed")
        # self.task.add(close_hand)

        # 8. Attach Object
        # attach_object = stages.ModifyPlanningScene("attach object")
        # self.task.add(attach_object)

        # 9. Lift Object
        lift = stages.MoveRelative("lift object", self.arm_group)
        lift.properties.configureInitFrom(core.Stage.PARENT)
        lift.setMinMaxDistance(0.05, 0.15)
        lift.setIKFrame(self.hand_frame)

        # Set lift direction (upward in z)
        lift.setDirection(Vector3(x=0.0, y=0.0, z=1.0))
        self.task.add(lift)

        # 10. Move to Place Location
        move_to_place = stages.Connect(
            "move to place",
            stages.Connect.GroupPlannerVector([
                (self.arm_group, "RRTConnect")
            ])
        )
        move_to_place.setTimeout(5.0)
        move_to_place.properties.configureInitFrom(core.Stage.PARENT)
        self.task.add(move_to_place)

        # 11. Lower Object
        lower = stages.MoveRelative("lower object", self.arm_group)
        lower.properties.configureInitFrom(core.Stage.PARENT)
        lower.setMinMaxDistance(0.05, 0.15)
        lower.setIKFrame(self.hand_frame)

        # Set lower direction (downward in z)
        lower.setDirection(Vector3(x=0.0, y=0.0, z=-1.0))
        self.task.add(lower)

        # 12. Detach Object
        # detach_object = stages.ModifyPlanningScene("detach object")
        # self.task.add(detach_object)

        # 13. Open Hand
        # open_hand_place = stages.MoveTo("open hand", self.hand_group)
        # open_hand_place.setGoal("open")
        # self.task.add(open_hand_place)

        # 14. Retreat from Object
        retreat = stages.MoveRelative("retreat", self.arm_group)
        retreat.properties.configureInitFrom(core.Stage.PARENT)
        retreat.setMinMaxDistance(0.05, 0.15)
        retreat.setIKFrame(self.hand_frame)

        # Set retreat direction (upward in z)
        retreat.setDirection(Vector3(x=0.0, y=0.0, z=1.0))
        self.task.add(retreat)

        # 15. Return to Home
        move_to_home = stages.MoveTo("move to home", self.arm_group)
        move_to_home.setGoal("home")  # Predefined pose in SRDF
        self.task.add(move_to_home)

        self.get_logger().info("Task pipeline configured successfully")

    def plan(self):
        """Plan the task"""
        self.get_logger().info("Planning task...")

        try:
            success = self.task.plan()
            if success:
                self.get_logger().info("Task planning succeeded!")
                return True
            else:
                self.get_logger().error("Task planning failed!")
                return False
        except Exception as e:
            self.get_logger().error(f"Planning error: {str(e)}")
            return False

    def execute(self):
        """Execute the planned task"""
        self.get_logger().info("Executing task...")

        try:
            success = self.task.execute()
            if success:
                self.get_logger().info("Task execution succeeded!")
                return True
            else:
                self.get_logger().error("Task execution failed!")
                return False
        except Exception as e:
            self.get_logger().error(f"Execution error: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)

    pick_place = PickPlaceTask()

    # Setup the task pipeline
    pick_place.setup_task()

    # Plan the task
    if pick_place.plan():
        # Ask user to execute
        pick_place.get_logger().info("Plan successful! Execute? (y/n)")
        # In real scenario, would wait for confirmation
        # For demo, auto-execute
        pick_place.execute()

    rclpy.spin(pick_place)
    pick_place.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
