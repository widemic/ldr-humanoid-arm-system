#!/usr/bin/env python3
"""
Keyboard teleoperation client for the LDR humanoid arm.

This node reads keystrokes from the terminal, keeps track of the desired joint
targets, and publishes `sensor_msgs/msg/JointState` commands for the C++ teleop
node (`joint_teleop_node`).
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty
from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState


HELP_TEXT = """
Arm Keyboard Teleop
===================
 Joint jogging (step size configurable via `step` parameter):
   q/a : shoulder_pitch_joint up/down
   w/s : shoulder_roll_joint forward/back
   e/d : shoulder_yaw_joint rotate out/in
   r/f : elbow_pitch_joint bend/extend
   t/g : elbow_yaw_joint rotate

 Preset poses:
   1 : home
   2 : ready
   3 : vertical

 Other:
   c : capture current joint state as the teleop target
   space : hold position (re-send current target)
   h or ? : show this help
   Ctrl+C / Esc : quit
"""


class KeyboardReader:
    """Read single characters from stdin without waiting for newline."""

    def __enter__(self) -> "KeyboardReader":
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires a TTY. Run in a terminal.")
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_key(self, timeout: float = 0.1) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return None


class KeyboardTeleop(Node):
    """Publish JointState commands based on keyboard input."""

    JOINT_NAMES = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "elbow_yaw_joint",
        "wrist_roll_joint",
    ]
    JOINT_LABELS = [
        "Shoulder pitch",
        "Shoulder roll",
        "Shoulder yaw",
        "Elbow pitch",
        "Elbow yaw",
        "Wrist roll",
    ]
    JOINT_INDEX = {name: idx for idx, name in enumerate(JOINT_NAMES)}

    KEY_BINDINGS = {
        "q": (0, 1.0),
        "a": (0, -1.0),
        "w": (1, 1.0),
        "s": (1, -1.0),
        "e": (2, 1.0),
        "d": (2, -1.0),
        "r": (3, 1.0),
        "f": (3, -1.0),
        "t": (4, 1.0),
        "g": (4, -1.0),
        "y": (5, 1.0),
        "h": (5, -1.0),
    }
    PRESET_POSES = {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "ready": [0.5, 0.5, 0.0, 0.5, 0.0, 0.0],
        "vertical": [1.0, 1.5, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self) -> None:
        super().__init__("arm_keyboard_teleop")

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
        self._step = float(self.declare_parameter("step", 0.05).value)

        self._command_pub = self.create_publisher(JointState, self._command_topic, 10)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self._state_callback,
            qos,
        )

        self._lock = threading.Lock()
        self._current_positions: Optional[List[float]] = None
        self._target_positions: Optional[List[float]] = None

    # ------------------------------------------------------------------ #
    def _state_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        positions: List[Optional[float]] = [None] * len(self.JOINT_NAMES)
        for name, position in zip(msg.name, msg.position):
            if name in self.JOINT_INDEX:
                positions[self.JOINT_INDEX[name]] = position

        if any(value is None for value in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]  # type: ignore[arg-type]
            if self._target_positions is None:
                self._target_positions = list(self._current_positions)

    def handle_key(self, key: str) -> None:
        if key in ("\n", "\r"):
            return
        if key in ("h", "?"):
            self.print_help()
            return
        if key == " ":
            self._resend_target()
            return
        if key == "c":
            self._capture_current_state()
            return
        if key in ("1", "2", "3"):
            pose_name = {"1": "home", "2": "ready", "3": "vertical"}[key]
            self._send_pose(pose_name)
            return
        if key in self.KEY_BINDINGS:
            joint_index, direction = self.KEY_BINDINGS[key]
            self._increment_joint(joint_index, direction * self._step)
            return

        if key.isprintable():
            self.get_logger().info(f"No action bound to '{key}'. Press 'h' for help.")

    # ------------------------------------------------------------------ #
    def _capture_current_state(self) -> None:
        with self._lock:
            if self._current_positions is None:
                self.get_logger().warn("Joint states not received yet.")
                return
            self._target_positions = list(self._current_positions)
            target = list(self._target_positions)
        self.get_logger().info("Captured current joint state as teleop target.")
        self._publish_target(target)

    def _increment_joint(self, index: int, delta: float) -> None:
        with self._lock:
            if self._target_positions is None:
                if self._current_positions is None:
                    self.get_logger().warn(
                        "Joint states not received yet. Unable to jog."
                    )
                    return
                self._target_positions = list(self._current_positions)

            self._target_positions[index] += delta
            target = list(self._target_positions)

        label = self.JOINT_LABELS[index]
        self.get_logger().info(
            f"Jogging {label} by {delta:+.3f} rad -> {target[index]:+.3f}"
        )
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

    def _resend_target(self) -> None:
        with self._lock:
            if self._target_positions is None:
                self.get_logger().warn("No target available to resend.")
                return
            target = list(self._target_positions)
        self.get_logger().info("Re-sending current target.")
        self._publish_target(target)

    def _publish_target(self, target: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(target)
        self._command_pub.publish(msg)

    def print_help(self) -> None:
        print(HELP_TEXT)
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    node.print_help()
    try:
        with KeyboardReader() as reader:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.0)
                key = reader.read_key(timeout=0.1)
                executor.spin_once(timeout_sec=0.0)

                if key is None:
                    continue
                if key in ("\x03", "\x1b"):  # Ctrl+C or Escape
                    break
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
