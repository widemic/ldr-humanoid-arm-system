#!/usr/bin/env python3
"""
Cartesian joystick teleoperation for the LDR humanoid arm.

This node uses MoveIt IK to convert joystick XYZ commands to joint positions,
then publishes them to the joint_teleop_node backend.
"""

from __future__ import annotations

import math
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.srv import GetPositionIK

import tf2_ros
from tf2_ros import TransformException


class CartesianJoystickTeleop(Node):
    """Convert joystick XYZ input to joint commands via MoveIt IK."""

    DEFAULT_PROFILE = "dualsense"
    JOINT_NAMES = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "elbow_yaw_joint",
        "wrist_roll_joint",
    ]

    CONTROLLER_PROFILES: Dict[str, Dict[str, any]] = {
        "xbox": {
            "axis_x": 1,      # Left stick Y
            "axis_y": 0,      # Left stick X
            "axis_z": 4,      # Right stick Y
            "axis_scales": [1.0, -1.0, 1.0],  # Invert Y for intuitive control
        },
        "dualsense": {
            "axis_x": 0,      # Left stick X (right=X+, left=X-)
            "axis_y": 1,      # Left stick Y (up=Y+, down=Y-)
            "axis_z": 3,      # Right stick Y (up=Z+, down=Z-)
            "axis_scales": [-1.0, 1.0, 1.0],  # Invert X only
        },
    }

    DEFAULT_BUTTONS = {
        "capture": 3,       # Triangle (DualSense) / Y (Xbox)
        "resend": 0,        # Cross (DualSense) / A (Xbox)
        "home": 4,          # L1 (DualSense) / LB (Xbox)
        "ready": 5,         # R1 (DualSense) / RB (Xbox)
        "vertical": 1,      # Circle (DualSense) / B (Xbox)
        "toggle_debug": 8,  # Share (DualSense) / View (Xbox)
    }

    # Joint positions for preset poses (6 DOF)
    HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Ready position - better for Cartesian control (not at singularity)
    READY_JOINTS = [0.5, 0.5, 0.0, -0.5, 0.0, 0.0]
    # Vertical position
    VERTICAL_JOINTS = [1.0, 1.5, 0.0, 0.0, 0.0, 0.0]

    def __init__(self) -> None:
        super().__init__("cartesian_joystick_teleop")

        # Parameters
        self._command_topic = (
            self.declare_parameter("command_topic", "teleop/joint_commands")
            .get_parameter_value()
            .string_value
        )
        joint_state_topic = (
            self.declare_parameter("joint_state_topic", "/joint_states")
            .get_parameter_value()
            .string_value
        )
        joy_topic = (
            self.declare_parameter("joy_topic", "/joy")
            .get_parameter_value()
            .string_value
        )
        self._debug_mode = bool(self.declare_parameter("debug_mode", False).value)
        self._cartesian_scale = float(
            self.declare_parameter("cartesian_scale", 0.1).value
        )
        self._update_rate = float(self.declare_parameter("update_rate", 30.0).value)
        self._update_rate = max(1.0, self._update_rate)
        self._update_period = 1.0 / self._update_rate
        self._deadzone = float(self.declare_parameter("deadzone", 0.15).value)
        self._base_frame = (
            self.declare_parameter("base_frame", "base_link")
            .get_parameter_value()
            .string_value
        )
        self._ee_frame = (
            self.declare_parameter("ee_frame", "end_effector_link")
            .get_parameter_value()
            .string_value
        )
        requested_profile = (
            self.declare_parameter("controller_profile", self.DEFAULT_PROFILE)
            .get_parameter_value()
            .string_value
        )

        # Load controller profile
        self._active_profile_name, profile = self._get_profile_defaults(requested_profile)
        self._axis_x = int(self.declare_parameter("axis_x", profile["axis_x"]).value)
        self._axis_y = int(self.declare_parameter("axis_y", profile["axis_y"]).value)
        self._axis_z = int(self.declare_parameter("axis_z", profile["axis_z"]).value)
        self._axis_scales = profile["axis_scales"]

        # Button configuration
        self._capture_button = int(
            self.declare_parameter("capture_button", self.DEFAULT_BUTTONS["capture"]).value
        )
        self._resend_button = int(
            self.declare_parameter("resend_button", self.DEFAULT_BUTTONS["resend"]).value
        )
        self._home_button = int(
            self.declare_parameter("home_button", self.DEFAULT_BUTTONS["home"]).value
        )
        self._ready_button = int(
            self.declare_parameter("ready_button", self.DEFAULT_BUTTONS["ready"]).value
        )
        self._vertical_button = int(
            self.declare_parameter("vertical_button", self.DEFAULT_BUTTONS["vertical"]).value
        )
        self._toggle_debug_button = int(
            self.declare_parameter("toggle_debug_button", self.DEFAULT_BUTTONS["toggle_debug"]).value
        )

        # QoS for joint states
        latched_joint_states = bool(
            self.declare_parameter("latched_joint_states", False).value
        )
        joint_state_durability = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if latched_joint_states
            else QoSDurabilityPolicy.VOLATILE
        )

        # Threading
        self._lock = threading.Lock()
        self._ik_lock = threading.Lock()

        # State
        self._current_joint_positions: Optional[List[float]] = None
        self._target_position: Optional[List[float]] = None  # [x, y, z]
        self._last_valid_position: Optional[List[float]] = None
        self._current_orientation: Optional[List[float]] = None  # [qx, qy, qz, qw]
        self._latest_axes: Optional[List[float]] = None
        self._previous_buttons: Optional[List[int]] = None
        self._warned_no_state = False
        self._tf_initialized = False
        self._ik_pending = False

        # TF2
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # IK service client
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Publishers
        self._command_pub = self.create_publisher(JointState, self._command_topic, 10)

        # Debug publishers (only used when debug_mode=True)
        self._target_pub = self.create_publisher(PointStamped, 'cartesian_teleop/target', 10)
        self._current_pub = self.create_publisher(PointStamped, 'cartesian_teleop/current', 10)

        # Subscribers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=joint_state_durability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self._state_callback,
            qos,
        )
        self._joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self._joy_callback,
            10,
        )

        # Timer for control loop
        self._timer = self.create_timer(self._update_period, self._timer_callback)

        self.get_logger().info(
            f"Cartesian joystick teleop ready (profile='{self._active_profile_name}'). "
            f"Listening to '{joy_topic}' and publishing commands on '{self._command_topic}'"
        )
        if self._debug_mode:
            self.get_logger().info("Debug mode enabled")

    def _get_profile_defaults(self, profile_name: str) -> Tuple[str, Dict]:
        key = (profile_name or self.DEFAULT_PROFILE).strip().lower()
        defaults = self.CONTROLLER_PROFILES.get(key)
        if defaults is None:
            self.get_logger().warn(
                f"Unknown controller_profile '{profile_name}'. Falling back to "
                f"'{self.DEFAULT_PROFILE}'."
            )
            key = self.DEFAULT_PROFILE
            defaults = self.CONTROLLER_PROFILES[key]
        return key, defaults

    def _state_callback(self, msg: JointState) -> None:
        """Store current joint positions for reference."""
        if not msg.position:
            return

        positions: List[Optional[float]] = [None] * len(self.JOINT_NAMES)
        joint_index = {name: idx for idx, name in enumerate(self.JOINT_NAMES)}

        for name, position in zip(msg.name, msg.position):
            idx = joint_index.get(name)
            if idx is None:
                continue
            positions[idx] = position

        if any(value is None for value in positions):
            return

        with self._lock:
            self._current_joint_positions = [float(p) for p in positions]
            self._warned_no_state = False

    def _joy_callback(self, msg: Joy) -> None:
        """Handle joystick input."""
        button_events: List[int] = []
        with self._lock:
            self._latest_axes = list(msg.axes)
            previous = self._previous_buttons or [0] * len(msg.buttons)
            for idx, state in enumerate(msg.buttons):
                prev_state = previous[idx] if idx < len(previous) else 0
                if state and not prev_state:
                    button_events.append(idx)
            self._previous_buttons = list(msg.buttons)

        for button_idx in button_events:
            self._handle_button_press(button_idx)

    def _handle_button_press(self, button_idx: int) -> None:
        """Handle button press events."""
        if button_idx == self._capture_button:
            self.get_logger().info("Capture button pressed - capturing current position")
            self._capture_current_position()
            return
        if button_idx == self._resend_button:
            self.get_logger().info("Resend button pressed - resending target")
            self._resend_target()
            return
        if button_idx == self._home_button:
            self.get_logger().info("Home button pressed - moving to home position")
            self._go_home()
            return
        if button_idx == self._ready_button:
            self.get_logger().info("Ready button pressed - moving to ready position")
            self._go_ready()
            return
        if button_idx == self._vertical_button:
            self.get_logger().info("Vertical button pressed - moving to vertical position")
            self._go_vertical()
            return
        if button_idx == self._toggle_debug_button:
            self._debug_mode = not self._debug_mode
            self.get_logger().info(f"Debug mode {'enabled' if self._debug_mode else 'disabled'}")
            return

    def _timer_callback(self) -> None:
        """Main control loop - convert joystick to Cartesian delta and compute IK."""
        # Skip if IK is already pending
        with self._ik_lock:
            if self._ik_pending:
                return

        # Initialize from TF if not done
        if not self._tf_initialized:
            self._initialize_from_tf()
            return

        axes_snapshot: Optional[List[float]] = None
        with self._lock:
            if self._target_position is None:
                return
            if self._latest_axes is None:
                return
            axes_snapshot = list(self._latest_axes)
            target = list(self._target_position)
            orientation = list(self._current_orientation) if self._current_orientation else None

        if orientation is None:
            return

        # Extract axis values with deadzone
        def get_axis(axis_idx: int, axes: List[float], scale: float) -> float:
            if axis_idx < 0 or axis_idx >= len(axes):
                return 0.0
            value = axes[axis_idx]
            if math.fabs(value) < self._deadzone:
                return 0.0
            return value * scale

        dx = get_axis(self._axis_x, axes_snapshot, self._axis_scales[0])
        dy = get_axis(self._axis_y, axes_snapshot, self._axis_scales[1])
        dz = get_axis(self._axis_z, axes_snapshot, self._axis_scales[2])

        # No movement requested
        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return

        # Compute delta
        delta_scale = self._cartesian_scale * self._update_period
        new_target = [
            target[0] + dx * delta_scale,
            target[1] + dy * delta_scale,
            target[2] + dz * delta_scale,
        ]

        # Call IK service
        self._compute_and_send_ik(new_target, orientation)

    def _initialize_from_tf(self) -> None:
        """Initialize target position and orientation from current TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]

            with self._lock:
                self._target_position = position
                self._last_valid_position = list(position)
                self._current_orientation = orientation
                self._tf_initialized = True

            self.get_logger().info(
                f"Initialized from TF: position=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
            )

        except TransformException as ex:
            if not self._warned_no_state:
                self.get_logger().warn(f"Waiting for TF: {ex}")
                self._warned_no_state = True

    def _capture_current_position(self) -> None:
        """Capture current end-effector position as new target."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]

            with self._lock:
                self._target_position = position
                self._last_valid_position = list(position)
                self._current_orientation = orientation

            self.get_logger().info(
                f"Captured position: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
            )

        except TransformException as ex:
            self.get_logger().error(f"Failed to capture position: {ex}")

    def _resend_target(self) -> None:
        """Resend current target position."""
        with self._lock:
            if self._target_position is None or self._current_orientation is None:
                self.get_logger().warn("No target available to resend")
                return
            target = list(self._target_position)
            orientation = list(self._current_orientation)

        self._compute_and_send_ik(target, orientation)

    def _go_home(self) -> None:
        """Send robot to home joint position."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(self.HOME_JOINTS)
        self._command_pub.publish(msg)
        self.get_logger().info("Sent home joint command")

        # Reset target position (will re-initialize from TF)
        with self._lock:
            self._tf_initialized = False
            self._target_position = None

    def _go_ready(self) -> None:
        """Send robot to ready position (better for Cartesian control)."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(self.READY_JOINTS)
        self._command_pub.publish(msg)
        self.get_logger().info("Sent ready joint command - wait for arm to move, then capture position")

        # Reset target position (will re-initialize from TF after arm moves)
        with self._lock:
            self._tf_initialized = False
            self._target_position = None

    def _go_vertical(self) -> None:
        """Send robot to vertical joint position."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(self.VERTICAL_JOINTS)
        self._command_pub.publish(msg)
        self.get_logger().info("Sent vertical joint command - wait for arm to move, then capture position")

        # Reset target position (will re-initialize from TF after arm moves)
        with self._lock:
            self._tf_initialized = False
            self._target_position = None

    def _compute_and_send_ik(self, target: List[float], orientation: List[float]) -> None:
        """Compute IK and send joint command."""
        with self._ik_lock:
            self._ik_pending = True

        # Check if IK service is available
        if not self._ik_client.service_is_ready():
            if self._debug_mode:
                self.get_logger().warn("IK service not available - is move_group running?")
            with self._ik_lock:
                self._ik_pending = False
            return

        if self._debug_mode:
            self.get_logger().debug(
                f"IK request: pos=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) "
                f"orient=({orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f})"
            )

        # Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = False  # Disable for now - can cause failures
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 100000000  # 100ms timeout

        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.position.z = target[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        req.ik_request.pose_stamped = pose

        # Use current joint state as seed to help IK solver
        with self._lock:
            if self._current_joint_positions:
                seed_state = JointState()
                seed_state.name = list(self.JOINT_NAMES)
                seed_state.position = list(self._current_joint_positions)
                req.ik_request.robot_state.joint_state = seed_state

        # Get current EE position for debug output
        current_pos = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            current_pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        except TransformException:
            pass

        # Call IK service asynchronously
        future = self._ik_client.call_async(req)
        future.add_done_callback(
            lambda f: self._ik_callback(f, target, current_pos)
        )

    def _ik_callback(
        self,
        future,
        target: List[float],
        current_pos: Optional[List[float]],
    ) -> None:
        """Handle IK service response."""
        try:
            result = future.result()
            # MoveIt error code 1 = SUCCESS
            ik_success = result.error_code.val == 1

            if ik_success:
                # Extract joint positions from solution
                solution = result.solution.joint_state
                joint_positions = self._extract_joint_positions(solution)

                if joint_positions:
                    # Update target position
                    with self._lock:
                        self._target_position = list(target)
                        self._last_valid_position = list(target)

                    # Publish joint command
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = list(self.JOINT_NAMES)
                    msg.position = joint_positions
                    self._command_pub.publish(msg)

                    if self._debug_mode:
                        current_str = (
                            f"({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})"
                            if current_pos else "N/A"
                        )
                        joints_str = ", ".join([f"{p:.2f}" for p in joint_positions])
                        self.get_logger().info(
                            f"[DEBUG] Target XYZ: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) | "
                            f"Current: {current_str} | IK: OK | Joints: [{joints_str}]"
                        )
                        # Publish target and current for PlotJuggler
                        self._publish_debug_points(target, current_pos)
                else:
                    self._handle_ik_failure(target, current_pos, "Joint extraction failed")
            else:
                self._handle_ik_failure(target, current_pos, f"Error code: {result.error_code.val}")

        except Exception as e:
            self._handle_ik_failure(target, current_pos, str(e))

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

    def _handle_ik_failure(
        self,
        target: List[float],
        current_pos: Optional[List[float]],
        reason: str,
    ) -> None:
        """Handle IK failure - revert target and log."""
        with self._lock:
            if self._last_valid_position:
                self._target_position = list(self._last_valid_position)

        if self._debug_mode:
            current_str = (
                f"({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})"
                if current_pos else "N/A"
            )
            self.get_logger().warn(
                f"[DEBUG] Target XYZ: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) | "
                f"Current: {current_str} | IK: FAILED ({reason})"
            )
            # Publish target and current for PlotJuggler
            self._publish_debug_points(target, current_pos)

    def _publish_debug_points(
        self,
        target: List[float],
        current_pos: Optional[List[float]],
    ) -> None:
        """Publish target and current positions for PlotJuggler visualization."""
        now = self.get_clock().now().to_msg()

        # Publish target position
        target_msg = PointStamped()
        target_msg.header.stamp = now
        target_msg.header.frame_id = self._base_frame
        target_msg.point.x = target[0]
        target_msg.point.y = target[1]
        target_msg.point.z = target[2]
        self._target_pub.publish(target_msg)

        # Publish current position
        if current_pos:
            current_msg = PointStamped()
            current_msg.header.stamp = now
            current_msg.header.frame_id = self._base_frame
            current_msg.point.x = current_pos[0]
            current_msg.point.y = current_pos[1]
            current_msg.point.z = current_pos[2]
            self._current_pub.publish(current_msg)


def main() -> None:
    rclpy.init()
    node = CartesianJoystickTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
