#!/usr/bin/env python3
"""
MoveIt Task Constructor Pick and Place Demo

This script demonstrates using MoveIt Task Constructor (MTC) to perform
a pick-and-place task with the 5-DOF humanoid arm.

MTC allows you to define manipulation tasks as a sequence of stages:
- Approach object
- Grasp object
- Lift object
- Move to target
- Place object
- Retreat

Author: LDR Robotics Team
License: MIT
"""

import sys

# Check if MTC is available
try:
    from moveit.task_constructor import core, stages
    from moveit.core.robot_state import RobotState
except ImportError as e:
    print("\n" + "="*60)
    print("ERROR: MoveIt Task Constructor not installed!")
    print("="*60)
    print(f"\nImport error: {e}")
    print("\nTo install MTC, run:")
    print("  sudo apt install ros-humble-moveit-task-constructor-*")
    print("\nFor detailed installation instructions, run:")
    print("  ros2 run arm_demos check_mtc.py")
    print("\n" + "="*60)
    sys.exit(1)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, CollisionObject


class MTCPickPlaceDemo(Node):
    """Node for demonstrating MoveIt Task Constructor pick and place."""

    def __init__(self):
        super().__init__('mtc_pick_place_demo')

        # MTC task
        self.task = None

        # Configuration
        self.arm_group_name = "arm"
        self.hand_group_name = "hand"  # Will be used when gripper is added
        self.eef_name = "tcp"  # End-effector name (update when gripper added)
        self.hand_frame = "wrist_roll_link"  # Current end-effector link

        # Object parameters
        self.object_name = "target_object"
        self.surface_name = "table"

        self.get_logger().info("MTC Pick and Place Demo initialized")

    def setup_planning_scene(self):
        """Add collision objects to the planning scene."""
        # This would typically use PlanningSceneInterface
        # For now, we'll define the objects in the task
        pass

    def create_task(self):
        """Create the MTC task with all stages."""
        self.task = core.Task("pick_place_task")

        # Set task properties
        self.task.loadRobotModel(self.get_logger())

        # Get the robot model
        robot_model = self.task.getRobotModel()

        # Set task properties
        self.task.setProperty("group", self.arm_group_name)
        self.task.setProperty("eef", self.eef_name)
        self.task.setProperty("ik_frame", self.hand_frame)

        # ==========================
        # Create planning pipelines
        # ==========================

        # Sampling planner for complex motions
        sampling_planner = core.PipelinePlanner()
        sampling_planner.setProperty("goal_joint_tolerance", 1e-5)

        # Cartesian planner for linear motions
        cartesian_planner = core.CartesianPath()
        cartesian_planner.setMaxVelocityScalingFactor(1.0)
        cartesian_planner.setMaxAccelerationScalingFactor(1.0)
        cartesian_planner.setStepSize(0.01)

        # Interpolation planner for simple joint movements
        interpolation_planner = core.JointInterpolation()

        # ==========================
        # Stage 1: Current State
        # ==========================
        current_state = stages.CurrentState("current state")
        self.task.add(current_state)

        # ==========================
        # Stage 2: Open Hand (when gripper available)
        # ==========================
        # TODO: Enable when gripper/hand is integrated
        # open_hand = stages.MoveTo("open hand", interpolation_planner)
        # open_hand.setGroup(self.hand_group_name)
        # open_hand.setGoal("open")
        # self.task.add(open_hand)

        # ==========================
        # Stage 3: Move to Pick Position
        # ==========================
        pick_container = core.SerialContainer("pick object")
        self.task.add(pick_container)

        # 3a: Approach object
        approach = stages.MoveRelative("approach object", cartesian_planner)
        approach.setGroup(self.arm_group_name)

        # Set direction (move down in Z)
        approach.setMinMaxDistance(0.05, 0.15)

        # Define approach direction as Vector3Stamped
        direction = Vector3Stamped()
        direction.header.frame_id = "base_link"
        direction.vector.z = -1.0  # Move down
        approach.setDirection(direction)

        pick_container.insert(approach)

        # 3b: Generate grasp pose
        grasp_generator = stages.GenerateGraspPose("generate grasp pose")
        grasp_generator.setAngleDelta(0.2)  # Angular resolution for sampling
        grasp_generator.setPreGraspPose("open")  # Hand posture before grasp
        grasp_generator.setGraspPose("closed")   # Hand posture during grasp
        grasp_generator.setMonitoredStage(current_state)

        # Define object pose
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.position.x = 0.3
        object_pose.pose.position.y = 0.0
        object_pose.pose.position.z = 0.2
        object_pose.pose.orientation.w = 1.0

        grasp_generator.setObject(self.object_name)
        grasp_generator.setPose(object_pose)

        # Compute IK for grasp
        grasp_ik = stages.ComputeIK("grasp pose IK", grasp_generator)
        grasp_ik.setMaxIKSolutions(8)
        grasp_ik.setIKFrame(self.hand_frame)
        grasp_ik.setTargetPose(object_pose)
        grasp_ik.properties().configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)

        pick_container.insert(grasp_ik)

        # 3c: Allow collision with object (approach stage)
        allow_touch = stages.ModifyPlanningScene("allow collision (hand-object)")
        allow_touch.allowCollisions(
            self.object_name,
            [self.hand_frame],  # Links that can touch the object
            True
        )
        pick_container.insert(allow_touch)

        # 3d: Close hand (when gripper available)
        # TODO: Enable when gripper/hand is integrated
        # close_hand = stages.MoveTo("close hand", interpolation_planner)
        # close_hand.setGroup(self.hand_group_name)
        # close_hand.setGoal("closed")
        # pick_container.insert(close_hand)

        # 3e: Attach object
        attach_object = stages.ModifyPlanningScene("attach object")
        attach_object.attachObject(self.object_name, self.hand_frame)
        pick_container.insert(attach_object)

        # 3f: Lift object
        lift = stages.MoveRelative("lift object", cartesian_planner)
        lift.setGroup(self.arm_group_name)
        lift.setMinMaxDistance(0.08, 0.15)

        # Lift direction (up in Z)
        lift_direction = Vector3Stamped()
        lift_direction.header.frame_id = "base_link"
        lift_direction.vector.z = 1.0  # Move up
        lift.setDirection(lift_direction)

        pick_container.insert(lift)

        # ==========================
        # Stage 4: Move to Place Position
        # ==========================

        # 4a: Move to pre-place pose
        move_to_place = stages.Connect(
            "move to place",
            [(self.arm_group_name, sampling_planner)]
        )
        move_to_place.properties().configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
        self.task.add(move_to_place)

        # ==========================
        # Stage 5: Place Object
        # ==========================
        place_container = core.SerialContainer("place object")
        self.task.add(place_container)

        # 5a: Lower object
        lower = stages.MoveRelative("lower object", cartesian_planner)
        lower.setGroup(self.arm_group_name)
        lower.setMinMaxDistance(0.05, 0.15)

        # Lower direction (down in Z)
        lower_direction = Vector3Stamped()
        lower_direction.header.frame_id = "base_link"
        lower_direction.vector.z = -1.0  # Move down
        lower.setDirection(lower_direction)

        place_container.insert(lower)

        # 5b: Generate place pose
        place_generator = stages.GeneratePlacePose("generate place pose")
        place_generator.setObject(self.object_name)

        # Define place location
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = 0.3
        place_pose.pose.position.y = 0.3
        place_pose.pose.position.z = 0.2
        place_pose.pose.orientation.w = 1.0

        place_generator.setPose(place_pose)
        place_generator.setMonitoredStage(current_state)

        place_container.insert(place_generator)

        # 5c: Open hand (when gripper available)
        # TODO: Enable when gripper/hand is integrated
        # open_hand_place = stages.MoveTo("open hand", interpolation_planner)
        # open_hand_place.setGroup(self.hand_group_name)
        # open_hand_place.setGoal("open")
        # place_container.insert(open_hand_place)

        # 5d: Detach object
        detach_object = stages.ModifyPlanningScene("detach object")
        detach_object.detachObject(self.object_name, self.hand_frame)
        place_container.insert(detach_object)

        # 5e: Forbid collision with object
        forbid_touch = stages.ModifyPlanningScene("forbid collision (hand-object)")
        forbid_touch.allowCollisions(
            self.object_name,
            [self.hand_frame],
            False
        )
        place_container.insert(forbid_touch)

        # 5f: Retreat
        retreat = stages.MoveRelative("retreat", cartesian_planner)
        retreat.setGroup(self.arm_group_name)
        retreat.setMinMaxDistance(0.05, 0.15)

        # Retreat direction (up in Z)
        retreat_direction = Vector3Stamped()
        retreat_direction.header.frame_id = "base_link"
        retreat_direction.vector.z = 1.0  # Move up
        retreat.setDirection(retreat_direction)

        place_container.insert(retreat)

        # ==========================
        # Stage 6: Return Home
        # ==========================
        return_home = stages.MoveTo("return home", sampling_planner)
        return_home.setGroup(self.arm_group_name)
        return_home.setGoal("home")  # Uses named pose from SRDF
        self.task.add(return_home)

        self.get_logger().info("MTC task created with all stages")

    def plan_task(self):
        """Plan the MTC task."""
        if self.task is None:
            self.get_logger().error("Task not created. Call create_task() first.")
            return False

        self.get_logger().info("Planning MTC task...")

        try:
            # Plan the task
            self.task.plan()

            # Check if planning succeeded
            if self.task.numSolutions() == 0:
                self.get_logger().error("No solutions found!")
                return False

            self.get_logger().info(f"Found {self.task.numSolutions()} solution(s)")
            return True

        except Exception as e:
            self.get_logger().error(f"Planning failed: {str(e)}")
            return False

    def execute_task(self):
        """Execute the planned MTC task."""
        if self.task is None:
            self.get_logger().error("Task not created.")
            return False

        if self.task.numSolutions() == 0:
            self.get_logger().error("No solutions to execute. Plan first.")
            return False

        self.get_logger().info("Executing MTC task...")

        try:
            # Execute the best solution
            self.task.execute(self.task.solutions()[0])
            self.get_logger().info("Task execution completed!")
            return True

        except Exception as e:
            self.get_logger().error(f"Execution failed: {str(e)}")
            return False

    def introspect_task(self):
        """Print task structure for debugging."""
        if self.task is None:
            self.get_logger().error("Task not created.")
            return

        self.get_logger().info("Task introspection:")
        self.get_logger().info(self.task.toString())


def main(args=None):
    """Main function to run the MTC demo."""
    rclpy.init(args=args)

    demo = MTCPickPlaceDemo()

    try:
        # Setup planning scene (add objects)
        demo.setup_planning_scene()

        # Create the task
        demo.create_task()

        # Print task structure
        demo.introspect_task()

        # Plan the task
        if demo.plan_task():
            # Ask user if they want to execute
            response = input("\nExecute the planned task? (y/n): ")
            if response.lower() == 'y':
                demo.execute_task()

        demo.get_logger().info("Demo completed")

    except KeyboardInterrupt:
        demo.get_logger().info("Demo interrupted by user")
    except Exception as e:
        demo.get_logger().error(f"Demo failed: {str(e)}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
