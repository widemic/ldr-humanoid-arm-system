#!/usr/bin/env python3
"""
PyQt5 GUI teleoperation client for the LDR humanoid arm.

Publishes `sensor_msgs/msg/JointState` commands that are consumed by the
`joint_teleop_node` backend.
"""

from __future__ import annotations

import subprocess
import sys
import threading
from typing import List, Optional

import rclpy
from PyQt5 import QtCore, QtWidgets
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState


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
POSES = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.5, 0.5, 0.0, 0.5, 0.0, 0.0],
    "vertical": [1.0, 1.5, 0.0, 0.0, 0.0, 0.0],
}


class ArmTeleopInterface(Node):
    """ROS interface used by the GUI."""

    def __init__(self) -> None:
        super().__init__("arm_gui_teleop")

        command_topic = (
            self.declare_parameter("command_topic", "teleop/joint_commands")
            .get_parameter_value()
            .string_value
        )
        joint_state_topic = (
            self.declare_parameter("joint_state_topic", "/joint_states")
            .get_parameter_value()
            .string_value
        )

        self._lock = threading.Lock()
        self._current_positions: Optional[List[float]] = None
        self._target_positions: Optional[List[float]] = None
        self._status = "Waiting for joint states..."

        self._command_pub = self.create_publisher(JointState, command_topic, 10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            qos,
        )

    # ------------------------------------------------------------------ #
    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        positions = [None] * len(JOINT_NAMES)
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_INDEX:
                positions[JOINT_INDEX[name]] = position

        if any(value is None for value in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]  # type: ignore[arg-type]
            if self._target_positions is None:
                self._target_positions = list(self._current_positions)
            self._status = "Joint states received."

    # ------------------------------------------------------------------ #
    def send_jog(self, joint_index: int, delta: float) -> None:
        with self._lock:
            if not self._ensure_target_locked():
                return
            self._target_positions[joint_index] += delta
            target = list(self._target_positions)
            label = JOINT_LABELS[joint_index]
            self._status = (
                f"Jogging {label} by {delta:+.3f} rad "
                f"(-> {target[joint_index]:+.3f})"
            )
        self._publish_target(target)

    def send_pose(self, pose_name: str) -> None:
        pose = POSES.get(pose_name)
        if not pose:
            with self._lock:
                self._status = f"Pose '{pose_name}' not defined."
            return
        with self._lock:
            self._target_positions = list(pose)
            target = list(self._target_positions)
            self._status = f"Moving to '{pose_name}' pose."
        self._publish_target(target)

    def send_capture(self) -> None:
        with self._lock:
            if self._current_positions is None:
                self._status = "Joint states not received yet."
                return
            self._target_positions = list(self._current_positions)
            target = list(self._target_positions)
            self._status = "Captured current joint state."
        self._publish_target(target)

    def send_resend(self) -> None:
        with self._lock:
            if self._target_positions is None:
                self._status = "No target stored yet."
                return
            target = list(self._target_positions)
            self._status = "Re-sending current target."
        self._publish_target(target)

    def _ensure_target_locked(self) -> bool:
        if self._target_positions is None:
            if self._current_positions is None:
                self._status = "Joint states not received yet."
                return False
            self._target_positions = list(self._current_positions)
        return True

    def _publish_target(self, positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(positions)
        self._command_pub.publish(msg)

    # Data getters ----------------------------------------------------- #
    def get_joint_values(self) -> Optional[List[float]]:
        with self._lock:
            return list(self._current_positions) if self._current_positions else None

    def get_target_values(self) -> Optional[List[float]]:
        with self._lock:
            return list(self._target_positions) if self._target_positions else None

    def get_status(self) -> str:
        with self._lock:
            return self._status


class TeleopWindow(QtWidgets.QMainWindow):
    """Main window replicating the teleop bindings with buttons."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LDR Arm Teleop")
        self.resize(600, 400)

        self.executor = SingleThreadedExecutor()
        self.node = ArmTeleopInterface()
        self.executor.add_node(self.node)

        self._step = 0.600
        self._backend_process: Optional[subprocess.Popen] = None
        self._joystick_launch_process: Optional[subprocess.Popen] = None

        self._build_ui()
        self._connect_signals()
        self._start_timers()

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Parameter controls
        param_group = QtWidgets.QGroupBox("Parameters")
        form = QtWidgets.QFormLayout(param_group)
        self.step_spin = QtWidgets.QDoubleSpinBox()
        self.step_spin.setDecimals(3)
        self.step_spin.setRange(0.001, 1.0)
        self.step_spin.setValue(self._step)
        form.addRow("Step (rad)", self.step_spin)
        layout.addWidget(param_group)

        # Joint controls
        joint_group = QtWidgets.QGroupBox("Joint Jogging")
        joint_layout = QtWidgets.QGridLayout(joint_group)
        joint_layout.setHorizontalSpacing(8)
        joint_layout.setVerticalSpacing(6)

        self.joint_labels: List[QtWidgets.QLabel] = []
        for idx, (label_text, joint_name) in enumerate(zip(JOINT_LABELS, JOINT_NAMES)):
            minus_btn = QtWidgets.QPushButton("−")
            plus_btn = QtWidgets.QPushButton("+")
            minus_btn.setProperty("joint_index", idx)
            minus_btn.setProperty("direction", -1.0)
            plus_btn.setProperty("joint_index", idx)
            plus_btn.setProperty("direction", 1.0)

            value_label = QtWidgets.QLabel("cur: ---.---   tgt: ---.---")
            self.joint_labels.append(value_label)

            joint_layout.addWidget(QtWidgets.QLabel(label_text), idx, 0)
            joint_layout.addWidget(QtWidgets.QLabel(joint_name), idx, 1)
            joint_layout.addWidget(minus_btn, idx, 2)
            joint_layout.addWidget(plus_btn, idx, 3)
            joint_layout.addWidget(value_label, idx, 4)

        layout.addWidget(joint_group)

        # Pose buttons
        pose_group = QtWidgets.QGroupBox("Poses")
        pose_layout = QtWidgets.QHBoxLayout(pose_group)
        self.pose_buttons = {}
        for pose in ["home", "ready", "vertical"]:
            btn = QtWidgets.QPushButton(pose.capitalize())
            btn.setProperty("pose_name", pose)
            pose_layout.addWidget(btn)
            self.pose_buttons[pose] = btn
        layout.addWidget(pose_group)

        # Utility buttons
        utility_widget = QtWidgets.QWidget()
        utility_layout = QtWidgets.QHBoxLayout(utility_widget)
        self.capture_btn = QtWidgets.QPushButton("Capture current state")
        self.resend_btn = QtWidgets.QPushButton("Hold / resend target")
        self.backend_btn = QtWidgets.QPushButton("Start Joint Teleop Backend")
        self.joystick_launch_btn = QtWidgets.QPushButton(
            "Start Joystick Teleop Launch"
        )
        utility_layout.addWidget(self.capture_btn)
        utility_layout.addWidget(self.resend_btn)
        utility_layout.addWidget(self.backend_btn)
        utility_layout.addWidget(self.joystick_launch_btn)
        layout.addWidget(utility_widget)

        self.status_label = QtWidgets.QLabel("Status: starting...")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    # ------------------------------------------------------------------ #
    def _connect_signals(self) -> None:
        for button in self.findChildren(QtWidgets.QPushButton):
            if button.property("joint_index") is not None:
                button.clicked.connect(self._handle_joint_button)
            elif button.property("pose_name") is not None:
                button.clicked.connect(self._handle_pose_button)

        self.capture_btn.clicked.connect(self.node.send_capture)
        self.resend_btn.clicked.connect(self.node.send_resend)
        self.backend_btn.clicked.connect(self._toggle_backend)
        self.joystick_launch_btn.clicked.connect(self._toggle_joystick_launch)
        self.step_spin.valueChanged.connect(self._update_step)

    def _start_timers(self) -> None:
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(20)

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_joint_labels)
        self.ui_timer.start(200)

    # ------------------------------------------------------------------ #
    def _update_step(self, value: float) -> None:
        self._step = max(0.001, float(value))

    def _handle_joint_button(self) -> None:
        button = self.sender()
        if not isinstance(button, QtWidgets.QPushButton):
            return
        index = int(button.property("joint_index"))
        direction = float(button.property("direction"))
        delta = direction * self._step
        self.node.send_jog(index, delta)

    def _handle_pose_button(self) -> None:
        button = self.sender()
        if not isinstance(button, QtWidgets.QPushButton):
            return
        pose = button.property("pose_name")
        if pose:
            self.node.send_pose(pose)

    def _toggle_backend(self) -> None:
        if self._is_backend_running():
            self._stop_backend()
        else:
            self._start_backend()
        self._update_backend_button()

    def _toggle_joystick_launch(self) -> None:
        if self._is_joystick_launch_running():
            self._stop_joystick_launch()
        else:
            self._start_joystick_launch()
        self._update_joystick_button()

    def _start_backend(self) -> None:
        if self._is_backend_running():
            return
        try:
            self._backend_process = subprocess.Popen(
                ["ros2", "run", "arm_teleop", "joint_teleop_node"]
            )
        except FileNotFoundError:
            QtWidgets.QMessageBox.critical(
                self,
                "ros2 not found",
                "Could not locate the 'ros2' executable. Ensure the ROS 2 environment is sourced.",
            )
            self._backend_process = None
        except Exception as exc:  # pragma: no cover - defensive
            QtWidgets.QMessageBox.critical(
                self, "Failed to launch", f"Failed to start joint_teleop_node:\n{exc}"
            )
            self._backend_process = None

    def _stop_backend(self) -> None:
        if not self._backend_process:
            return
        if self._backend_process.poll() is None:
            self._backend_process.terminate()
            try:
                self._backend_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._backend_process.kill()
            except Exception:
                self._backend_process.kill()
        self._backend_process = None

    def _is_backend_running(self) -> bool:
        return (
            self._backend_process is not None
            and self._backend_process.poll() is None
        )

    def _update_backend_button(self) -> None:
        if not hasattr(self, "backend_btn"):
            return
        running = self._is_backend_running()
        text = (
            "Stop Joint Teleop Backend"
            if running
            else "Start Joint Teleop Backend"
        )
        self.backend_btn.setText(text)

    def _start_joystick_launch(self) -> None:
        if self._is_joystick_launch_running():
            return
        try:
            self._joystick_launch_process = subprocess.Popen(
                ["ros2", "launch", "arm_teleop", "joystick_teleop.launch.py"]
            )
        except FileNotFoundError:
            QtWidgets.QMessageBox.critical(
                self,
                "ros2 not found",
                "Could not locate the 'ros2' executable. Ensure the ROS 2 environment is sourced.",
            )
            self._joystick_launch_process = None
        except Exception as exc:  # pragma: no cover - defensive
            QtWidgets.QMessageBox.critical(
                self,
                "Failed to launch",
                f"Failed to start joystick_teleop launch:\n{exc}",
            )
            self._joystick_launch_process = None

    def _stop_joystick_launch(self) -> None:
        process = self._joystick_launch_process
        if not process:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            except Exception:
                process.kill()
        self._joystick_launch_process = None

    def _is_joystick_launch_running(self) -> bool:
        return (
            self._joystick_launch_process is not None
            and self._joystick_launch_process.poll() is None
        )

    def _update_joystick_button(self) -> None:
        if not hasattr(self, "joystick_launch_btn"):
            return
        running = self._is_joystick_launch_running()
        text = (
            "Stop Joystick Teleop Launch"
            if running
            else "Start Joystick Teleop Launch"
        )
        self.joystick_launch_btn.setText(text)

    def _spin_ros(self) -> None:
        self.executor.spin_once(timeout_sec=0.0)
        backend_state = "running" if self._is_backend_running() else "stopped"
        self.status_label.setText(
            "Status: "
            f"{self.node.get_status()} | Backend: {backend_state} | Joystick launch: "
            f"{'running' if self._is_joystick_launch_running() else 'stopped'}"
        )

    def _refresh_joint_labels(self) -> None:
        current = self.node.get_joint_values()
        target = self.node.get_target_values()
        for idx, label in enumerate(self.joint_labels):
            cur = (
                f"{current[idx]:+.3f}"
                if current and len(current) > idx
                else "---.---"
            )
            tgt = (
                f"{target[idx]:+.3f}"
                if target and len(target) > idx
                else "---.---"
            )
            label.setText(f"cur: {cur}   tgt: {tgt}")
        self._update_backend_button()
        self._update_joystick_button()

    def closeEvent(self, event) -> None:
        self.ros_timer.stop()
        self.ui_timer.stop()
        self._stop_backend()
        self._stop_joystick_launch()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        super().closeEvent(event)


def main() -> None:
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    window = TeleopWindow()
    window.show()
    try:
        app.exec_()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
