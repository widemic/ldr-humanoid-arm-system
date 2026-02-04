#!/usr/bin/env python3
"""
Visual Teleop for the LDR humanoid arm.

Uses skeleton tracking (MediaPipe + RealSense) to control the robot arm.
Subscribes to hand position from skeleton_tracker and converts to joint commands via MoveIt IK.
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from moveit_msgs.srv import GetPositionIK


class VisualTeleop(Node):
    """Control robot arm using skeleton hand tracking."""

    JOINT_NAMES = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "elbow_yaw_joint",
        "wrist_roll_joint",
    ]

    def __init__(self) -> None:
        super().__init__("visual_teleop")

        # Parameters
        self.declare_parameter("hand_topic", "/skeleton/hand_right")
        self.declare_parameter("command_topic", "teleop/joint_commands")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "end_effector_link")
        self.declare_parameter("enabled", True)
        self.declare_parameter("debug_mode", False)

        # Workspace mapping parameters (hand position -> robot position)
        self.declare_parameter("hand_workspace.x_min", 0.0)    # Hand workspace in camera frame
        self.declare_parameter("hand_workspace.x_max", 0.5)
        self.declare_parameter("hand_workspace.y_min", -0.3)
        self.declare_parameter("hand_workspace.y_max", 0.3)
        self.declare_parameter("hand_workspace.z_min", 0.3)
        self.declare_parameter("hand_workspace.z_max", 1.0)

        self.declare_parameter("robot_workspace.x_min", 0.1)   # Robot workspace in base_link frame
        self.declare_parameter("robot_workspace.x_max", 0.5)
        self.declare_parameter("robot_workspace.y_min", -0.3)
        self.declare_parameter("robot_workspace.y_max", 0.3)
        self.declare_parameter("robot_workspace.z_min", 0.1)
        self.declare_parameter("robot_workspace.z_max", 0.5)

        # Get parameters
        self._hand_topic = self.get_parameter("hand_topic").value
        self._command_topic = self.get_parameter("command_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._end_effector_frame = self.get_parameter("end_effector_frame").value
        self._enabled = self.get_parameter("enabled").value
        self._debug_mode = self.get_parameter("debug_mode").value

        # Workspace limits
        self._hand_ws = {
            "x_min": self.get_parameter("hand_workspace.x_min").value,
            "x_max": self.get_parameter("hand_workspace.x_max").value,
            "y_min": self.get_parameter("hand_workspace.y_min").value,
            "y_max": self.get_parameter("hand_workspace.y_max").value,
            "z_min": self.get_parameter("hand_workspace.z_min").value,
            "z_max": self.get_parameter("hand_workspace.z_max").value,
        }
        self._robot_ws = {
            "x_min": self.get_parameter("robot_workspace.x_min").value,
            "x_max": self.get_parameter("robot_workspace.x_max").value,
            "y_min": self.get_parameter("robot_workspace.y_min").value,
            "y_max": self.get_parameter("robot_workspace.y_max").value,
            "z_min": self.get_parameter("robot_workspace.z_min").value,
            "z_max": self.get_parameter("robot_workspace.z_max").value,
        }

        # State
        self._lock = threading.Lock()
        self._ik_lock = threading.Lock()
        self._ik_pending = False
        self._last_hand_position: Optional[List[float]] = None
        self._last_robot_target: Optional[List[float]] = None
        self._current_joint_positions: Optional[List[float]] = None

        # IK service client
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Publishers
        self._command_pub = self.create_publisher(JointState, self._command_topic, 10)

        # Debug publishers
        self._target_pub = self.create_publisher(PointStamped, 'visual_teleop/target', 10)
        self._hand_mapped_pub = self.create_publisher(PointStamped, 'visual_teleop/hand_mapped', 10)

        # Subscribers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._hand_sub = self.create_subscription(
            PointStamped,
            self._hand_topic,
            self._hand_callback,
            sensor_qos,
        )

        self._enable_sub = self.create_subscription(
            Bool,
            'visual_teleop/enable',
            self._enable_callback,
            10,
        )

        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        self.get_logger().info(
            f"Visual Teleop initialized\n"
            f"  Hand topic: {self._hand_topic}\n"
            f"  Command topic: {self._command_topic}\n"
            f"  Enabled: {self._enabled}\n"
            f"  Debug: {self._debug_mode}"
        )

    def _enable_callback(self, msg: Bool) -> None:
        """Enable/disable visual teleop."""
        self._enabled = msg.data
        self.get_logger().info(f"Visual teleop {'enabled' if self._enabled else 'disabled'}")

    def _joint_state_callback(self, msg: JointState) -> None:
        """Store current joint positions for IK seed."""
        positions = []
        for name in self.JOINT_NAMES:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])

        if len(positions) == len(self.JOINT_NAMES):
            with self._lock:
                self._current_joint_positions = positions

    def _hand_callback(self, msg: PointStamped) -> None:
        """Process hand position and compute robot target."""
        if not self._enabled:
            return

        # Check if IK is already pending
        with self._ik_lock:
            if self._ik_pending:
                return

        # Get hand position in camera frame
        hand_pos = [msg.point.x, msg.point.y, msg.point.z]

        with self._lock:
            self._last_hand_position = hand_pos

        # Map hand position to robot workspace
        robot_target = self._map_hand_to_robot(hand_pos)

        if robot_target is None:
            return

        with self._lock:
            self._last_robot_target = robot_target

        # Compute IK and send command
        self._compute_and_send_ik(robot_target)

    def _map_hand_to_robot(self, hand_pos: List[float]) -> Optional[List[float]]:
        """
        Map hand position from camera frame to robot workspace.

        Camera frame (RealSense):
          X = right, Y = down, Z = forward (away from camera)

        Robot frame (base_link):
          X = forward, Y = left, Z = up
        """
        hx, hy, hz = hand_pos

        # Clamp to hand workspace
        hx = max(self._hand_ws["x_min"], min(self._hand_ws["x_max"], hx))
        hy = max(self._hand_ws["y_min"], min(self._hand_ws["y_max"], hy))
        hz = max(self._hand_ws["z_min"], min(self._hand_ws["z_max"], hz))

        # Normalize to [0, 1]
        nx = (hx - self._hand_ws["x_min"]) / (self._hand_ws["x_max"] - self._hand_ws["x_min"])
        ny = (hy - self._hand_ws["y_min"]) / (self._hand_ws["y_max"] - self._hand_ws["y_min"])
        nz = (hz - self._hand_ws["z_min"]) / (self._hand_ws["z_max"] - self._hand_ws["z_min"])

        # Map to robot workspace with frame conversion:
        # Camera Z (forward) -> Robot X (forward)
        # Camera X (right) -> Robot -Y (left is positive)
        # Camera Y (down) -> Robot -Z (up is positive)
        robot_x = self._robot_ws["x_min"] + nz * (self._robot_ws["x_max"] - self._robot_ws["x_min"])
        robot_y = self._robot_ws["y_max"] - nx * (self._robot_ws["y_max"] - self._robot_ws["y_min"])
        robot_z = self._robot_ws["z_max"] - ny * (self._robot_ws["z_max"] - self._robot_ws["z_min"])

        return [robot_x, robot_y, robot_z]

    def _compute_and_send_ik(self, target: List[float]) -> None:
        """Compute IK and send joint command."""
        with self._ik_lock:
            self._ik_pending = True

        if not self._ik_client.service_is_ready():
            if self._debug_mode:
                self.get_logger().warn("IK service not available")
            with self._ik_lock:
                self._ik_pending = False
            return

        # Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 50000000  # 50ms

        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.position.z = target[2]
        # Use identity quaternion (position only IK)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = pose

        # Use current joint state as seed
        with self._lock:
            if self._current_joint_positions:
                seed_state = JointState()
                seed_state.name = list(self.JOINT_NAMES)
                seed_state.position = list(self._current_joint_positions)
                req.ik_request.robot_state.joint_state = seed_state

        # Call IK service
        future = self._ik_client.call_async(req)
        future.add_done_callback(
            lambda f: self._ik_callback(f, target)
        )

    def _ik_callback(self, future, target: List[float]) -> None:
        """Handle IK service response."""
        try:
            result = future.result()

            if result.error_code.val == 1:  # SUCCESS
                joint_positions = self._extract_joint_positions(result.solution.joint_state)

                if joint_positions:
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = list(self.JOINT_NAMES)
                    msg.position = joint_positions
                    self._command_pub.publish(msg)

                    if self._debug_mode:
                        self._publish_debug(target)
                        self.get_logger().info(
                            f"[DEBUG] Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) | IK: OK"
                        )
            else:
                if self._debug_mode:
                    self.get_logger().warn(
                        f"[DEBUG] Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) | "
                        f"IK: FAILED (code: {result.error_code.val})"
                    )
                    self._publish_debug(target)

        except Exception as e:
            if self._debug_mode:
                self.get_logger().error(f"IK callback error: {e}")

        finally:
            with self._ik_lock:
                self._ik_pending = False

    def _extract_joint_positions(self, joint_state: JointState) -> Optional[List[float]]:
        """Extract positions for our controlled joints from IK solution."""
        positions = [None] * len(self.JOINT_NAMES)
        joint_index = {name: idx for idx, name in enumerate(self.JOINT_NAMES)}

        for name, position in zip(joint_state.name, joint_state.position):
            idx = joint_index.get(name)
            if idx is not None:
                positions[idx] = position

        if any(p is None for p in positions):
            return None

        return [float(p) for p in positions]

    def _publish_debug(self, target: List[float]) -> None:
        """Publish debug information."""
        now = self.get_clock().now().to_msg()

        # Publish robot target
        target_msg = PointStamped()
        target_msg.header.stamp = now
        target_msg.header.frame_id = self._base_frame
        target_msg.point.x = target[0]
        target_msg.point.y = target[1]
        target_msg.point.z = target[2]
        self._target_pub.publish(target_msg)

        # Publish mapped hand position
        with self._lock:
            if self._last_hand_position:
                hand_msg = PointStamped()
                hand_msg.header.stamp = now
                hand_msg.header.frame_id = "camera_color_optical_frame"
                hand_msg.point.x = self._last_hand_position[0]
                hand_msg.point.y = self._last_hand_position[1]
                hand_msg.point.z = self._last_hand_position[2]
                self._hand_mapped_pub.publish(hand_msg)


def main() -> None:
    rclpy.init()
    node = VisualTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
