#!/usr/bin/env python3
"""
Joystick teleoperation client for the LDR humanoid arm.

This node consumes `sensor_msgs/msg/Joy` messages, keeps track of the desired
joint targets, and publishes `sensor_msgs/msg/JointState` commands for the
`joint_teleop_node` backend.
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


class JoystickTeleop(Node):
    """Publish JointState commands based on joystick input."""

    DEFAULT_PROFILE = "dualsense"
    JOINT_NAMES = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "elbow_yaw_joint",
        "wrist_roll_joint",
    ]
    JOINT_INDEX = {name: idx for idx, name in enumerate(JOINT_NAMES)}
    CONTROLLER_PROFILES: Dict[str, Dict[str, List[float]]] = {
        "xbox": {
            "axis_bindings": [1, 0, 3, 4, 2, 5],
            "axis_scales": [0.8, 0.6, 0.6, 0.8, 0.6, 0.6],
        },
        "dualsense": {
            "axis_bindings": [1, 0, 2, 3, 4, 5],
            "axis_scales": [5.0, 5.0, 5.0, 5.0, 5.0, 5.0],
        },
    }
    PRESET_POSES: Dict[str, List[float]] = {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "ready": [0.5, 0.5, 0.0, 0.5, 0.0, 0.0],
        "vertical": [1.0, 1.5, 0.0, 0.0, 0.0, 0.0],
    }
    DEFAULT_PRESET_BUTTONS = {
        "home": 4,      # LB
        "ready": 5,     # RB
        "vertical": 1,  # B
    }

    def __init__(self) -> None:
        super().__init__("arm_joystick_teleop")

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
        self._update_rate = float(self.declare_parameter("update_rate", 30.0).value)
        self._update_rate = max(1.0, self._update_rate)
        self._update_period = 1.0 / self._update_rate
        self._deadzone = float(self.declare_parameter("deadzone", 0.15).value)
        self._capture_button = int(self.declare_parameter("capture_button", 3).value)
        self._resend_button = int(self.declare_parameter("resend_button", 0).value)
        requested_profile = (
            self.declare_parameter("controller_profile", self.DEFAULT_PROFILE)
            .get_parameter_value()
            .string_value
        )
        self._active_profile_name, profile_defaults = self._get_profile_defaults(
            requested_profile
        )
        latched_joint_states = bool(
            self.declare_parameter("latched_joint_states", False).value
        )
        joint_state_durability = (
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if latched_joint_states
            else QoSDurabilityPolicy.VOLATILE
        )

        self._axis_bindings = self._load_numeric_list(
            "axis_bindings", profile_defaults["axis_bindings"], int
        )
        self._axis_scales = self._load_numeric_list(
            "axis_scales", profile_defaults["axis_scales"], float
        )
        self._align_list_lengths()
        self._preset_button_map = self._load_preset_buttons()

        self._lock = threading.Lock()
        self._current_positions: Optional[List[float]] = None
        self._target_positions: Optional[List[float]] = None
        self._latest_axes: Optional[List[float]] = None
        self._previous_buttons: Optional[List[int]] = None
        self._warned_no_state = False

        self._command_pub = self.create_publisher(JointState, self._command_topic, 10)
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
        self._timer = self.create_timer(self._update_period, self._timer_callback)

        self.get_logger().info(
            f"Joystick teleop ready (profile='{self._active_profile_name}'). Listening to "
            f"'{joy_topic}' and publishing commands on '{self._command_topic}'"
        )

    # ------------------------------------------------------------------ #
    def _load_numeric_list(
        self, name: str, default: List[float], value_type: type
    ) -> List[float]:
        value = self.declare_parameter(name, default).value
        if not isinstance(value, (list, tuple)):
            self.get_logger().warn(
                f"Parameter '{name}' expected a list. Falling back to default."
            )
            return list(default)
        try:
            return [value_type(item) for item in value]
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Parameter '{name}' had invalid entries. Falling back to default."
            )
            return list(default)

    def _get_profile_defaults(
        self, profile_name: str
    ) -> Tuple[str, Dict[str, List[float]]]:
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

    def _align_list_lengths(self) -> None:
        joint_count = len(self.JOINT_NAMES)
        if len(self._axis_bindings) != joint_count:
            self.get_logger().warn(
                "Parameter 'axis_bindings' length "
                f"({len(self._axis_bindings)}) does not match joint count ({joint_count}). "
                "It will be trimmed or padded with -1."
            )
            self._axis_bindings = self._resize_list(self._axis_bindings, joint_count, -1)
        if len(self._axis_scales) != joint_count:
            self.get_logger().warn(
                "Parameter 'axis_scales' length "
                f"({len(self._axis_scales)}) does not match joint count ({joint_count}). "
                "It will be trimmed or padded with 0.5."
            )
            self._axis_scales = self._resize_list(self._axis_scales, joint_count, 0.5)

    @staticmethod
    def _resize_list(values: List[float], target_len: int, filler: float) -> List[float]:
        if len(values) >= target_len:
            return list(values[:target_len])
        return list(values) + [filler] * (target_len - len(values))

    def _load_preset_buttons(self) -> Dict[int, str]:
        mapping: Dict[int, str] = {}
        for pose_name, default_button in self.DEFAULT_PRESET_BUTTONS.items():
            button = int(
                self.declare_parameter(
                    f"preset_buttons.{pose_name}", default_button
                ).value
            )
            if button >= 0:
                mapping[button] = pose_name
        return mapping

    # ------------------------------------------------------------------ #
    def _state_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        positions: List[Optional[float]] = [None] * len(self.JOINT_NAMES)
        for name, position in zip(msg.name, msg.position):
            idx = self.JOINT_INDEX.get(name)
            if idx is None:
                continue
            positions[idx] = position

        if any(value is None for value in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]  # type: ignore[arg-type]
            if self._target_positions is None:
                self._target_positions = list(self._current_positions)
            self._warned_no_state = False

    def _joy_callback(self, msg: Joy) -> None:
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
        if button_idx == self._capture_button:
            self.get_logger().info("Capture button pressed.")
            self._capture_current_state()
            return
        if button_idx == self._resend_button:
            self.get_logger().info("Hold button pressed.")
            self._resend_target()
            return
        pose = self._preset_button_map.get(button_idx)
        if pose:
            self.get_logger().info(f"Preset '{pose}' button pressed.")
            self._send_pose(pose)
            return
        self.get_logger().debug(f"Button {button_idx} has no binding.")

    def _timer_callback(self) -> None:
        axes_snapshot: Optional[List[float]] = None
        with self._lock:
            if not self._ensure_target_locked():
                return
            if self._latest_axes is None:
                return
            axes_snapshot = list(self._latest_axes)
            target = list(self._target_positions)

        updated = False
        for joint_index, axis_index in enumerate(self._axis_bindings):
            if axis_index < 0 or axis_index >= len(axes_snapshot):
                continue
            axis_value = axes_snapshot[axis_index]
            if math.fabs(axis_value) < self._deadzone:
                continue
            delta = axis_value * self._axis_scales[joint_index] * self._update_period
            target[joint_index] += delta
            updated = True

        if updated:
            with self._lock:
                if self._target_positions is not None:
                    self._target_positions = list(target)
            self._publish_target(target)

    def _ensure_target_locked(self) -> bool:
        if self._target_positions is None:
            if self._current_positions is None:
                if not self._warned_no_state:
                    self.get_logger().warn("Joint states not received yet.")
                    self._warned_no_state = True
                return False
            self._target_positions = list(self._current_positions)
        return True

    def _capture_current_state(self) -> None:
        with self._lock:
            if self._current_positions is None:
                self.get_logger().warn("Joint states not received yet.")
                return
            self._target_positions = list(self._current_positions)
            target = list(self._target_positions)
        self.get_logger().info("Captured current joint state as teleop target.")
        self._publish_target(target)

    def _resend_target(self) -> None:
        with self._lock:
            if self._target_positions is None:
                self.get_logger().warn("No target available to resend.")
                return
            target = list(self._target_positions)
        self.get_logger().info("Re-sending current target.")
        self._publish_target(target)

    def _send_pose(self, pose_name: str) -> None:
        pose = self.PRESET_POSES.get(pose_name)
        if not pose:
            self.get_logger().warn(f"Pose '{pose_name}' is not defined.")
            return
        with self._lock:
            self._target_positions = list(pose)
            target = list(self._target_positions)
        self.get_logger().info(f"Moving to '{pose_name}' pose.")
        self._publish_target(target)

    def _publish_target(self, target: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(target)
        self._command_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JoystickTeleop()
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
