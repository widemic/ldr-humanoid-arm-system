#!/usr/bin/env python3
"""
MoveIt Servo joystick teleoperation for the LDR humanoid arm.

Publishes TwistStamped commands to MoveIt Servo for real-time Cartesian control.

Controls (DualSense, hold L1 to enable):
  Left stick Y (up/down)      → Move forward/backward (X axis)
  Left stick X (left/right)   → Move left/right (Y axis)
  Right stick Y (up/down)     → Move up/down (Z axis)
  Right stick X (left/right)  → Yaw   (rotation around Z)
  D-pad up/down               → Roll  (rotation around X)
  D-pad left/right            → Pitch (rotation around Y)

Gripper:
  L2 (axis 2)  → Open gripper
  R2 (axis 5)  → Close gripper

Buttons:
  L1 (4)       → Enable servo (hold to move)
  R1 (5)       → Go to ready position
  Triangle (3) → Go to home position
  Share (8)    → Toggle debug mode
"""

from __future__ import annotations

import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import ServoCommandType


class ServoJoystickTeleop(Node):
    """Joystick teleop using MoveIt Servo."""

    JOINT_NAMES = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "elbow_yaw_joint",
        "wrist_roll_joint",
    ]

    HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    READY_JOINTS = [0.8, 0.8, 0.3, -0.8, 0.3, 0.0]

    COMMAND_TYPE_TWIST = 1

    def __init__(self) -> None:
        super().__init__("servo_joystick_teleop")

        self._callback_group = ReentrantCallbackGroup()

        # Parameters
        self._debug_mode = bool(self.declare_parameter("debug_mode", False).value)
        self._linear_scale = float(self.declare_parameter("linear_scale", 0.3).value)
        self._angular_scale = float(self.declare_parameter("angular_scale", 0.5).value)
        self._deadzone = float(self.declare_parameter("deadzone", 0.1).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)

        self._servo_topic = (
            self.declare_parameter("servo_topic", "/servo_node/delta_twist_cmds")
            .get_parameter_value()
            .string_value
        )
        self._frame_id = (
            self.declare_parameter("frame_id", "end_effector_link")
            .get_parameter_value()
            .string_value
        )

        # DualSense axis mapping
        # Left stick X (axis 0) → robot X (left=-X, right=+X)
        # Left stick Y (axis 1) → robot Y (up=+Y, down=-Y)
        # Right stick Y (axis 4) → robot Z (up=+Z, down=-Z)
        # Right stick X (axis 3) → yaw
        self._axis_x = 0          # Left stick X → robot X
        self._axis_y = 1          # Left stick Y → robot Y
        self._axis_z = 4          # Right stick Y → robot Z
        self._axis_yaw = 3        # Right stick X → yaw

        self._scale_x = -1.0      # left=-X, right=+X
        self._scale_y = 1.0       # up=+Y, down=-Y
        self._scale_z = 1.0       # up=+Z, down=-Z
        self._scale_yaw = -1.0

        # D-pad axis mapping (discrete: -1, 0, +1)
        self._axis_dpad_x = 6     # D-pad left/right → pitch
        self._axis_dpad_y = 7     # D-pad up/down → roll
        self._scale_roll = 1.0
        self._scale_pitch = 1.0

        # Trigger axis mapping (1.0=released, -1.0=fully pressed)
        self._axis_l2 = 2         # L2 → open gripper
        self._axis_r2 = 5         # R2 → close gripper
        self._trigger_threshold = 0.0  # pressed when axis < this

        # Gripper positions
        self._gripper_open = -0.04   # fully open
        self._gripper_closed = 0.0   # fully closed

        # Button mapping
        self._enable_button = 4   # L1 - enable servo
        self._ready_button = 5    # R1
        self._home_button = 3     # Triangle
        self._toggle_debug = 8    # Share

        # State
        self._lock = threading.Lock()
        self._latest_axes: Optional[List[float]] = None
        self._latest_buttons: Optional[List[int]] = None
        self._previous_buttons: Optional[List[int]] = None
        self._servo_ready = False
        self._servo_active = False
        self._command_type_set = False
        self._gripper_state: Optional[str] = None  # "open" or "closed"

        # Service client for setting command type
        self._switch_command_type_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
            callback_group=self._callback_group
        )

        # Publishers
        self._twist_pub = self.create_publisher(TwistStamped, self._servo_topic, 10)
        # Direct position commands to forward_command_controller
        self._position_pub = self.create_publisher(
            Float64MultiArray,
            "/servo_controller/commands",
            10
        )
        # Gripper trajectory commands to hand_controller
        self._gripper_pub = self.create_publisher(
            JointTrajectory,
            "/hand_controller/joint_trajectory",
            10
        )

        # Subscribers
        self._joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        # Timers
        self._timer = self.create_timer(1.0 / self._publish_rate, self._control_loop)
        self._init_timer = self.create_timer(1.0, self._init_servo)

        self.get_logger().info(
            f"Servo joystick teleop starting...\n"
            f"  Servo topic: {self._servo_topic}\n"
            f"  Linear scale: {self._linear_scale} m/s\n"
            f"  Hold L1 to enable servo control"
        )

    def _init_servo(self) -> None:
        """Initialize servo by setting command type."""
        if self._command_type_set:
            self._init_timer.cancel()
            return

        if not self._switch_command_type_client.service_is_ready():
            self.get_logger().info("Waiting for servo switch_command_type service...")
            return

        request = ServoCommandType.Request()
        request.command_type = self.COMMAND_TYPE_TWIST

        self.get_logger().info("Setting servo command type to TWIST...")
        future = self._switch_command_type_client.call_async(request)
        future.add_done_callback(self._command_type_callback)

    def _command_type_callback(self, future) -> None:
        """Handle command type response."""
        try:
            response = future.result()
            if response.success:
                self._command_type_set = True
                self._servo_ready = True
                self.get_logger().info("Servo ready! Hold L1 and move joystick.")
            else:
                self.get_logger().warn("Failed to set command type, retrying...")
        except Exception as e:
            self.get_logger().error(f"Command type service call failed: {e}")

    def _joy_callback(self, msg: Joy) -> None:
        """Handle joystick input."""
        button_events: List[int] = []

        with self._lock:
            self._latest_axes = list(msg.axes) if msg.axes else None
            self._latest_buttons = list(msg.buttons) if msg.buttons else None

            previous = self._previous_buttons or [0] * len(msg.buttons)
            for idx, state in enumerate(msg.buttons):
                prev_state = previous[idx] if idx < len(previous) else 0
                if state and not prev_state:
                    button_events.append(idx)
            self._previous_buttons = list(msg.buttons)

        for button_idx in button_events:
            self._handle_button(button_idx)

    def _handle_button(self, button_idx: int) -> None:
        """Handle button press."""
        if button_idx == self._home_button:
            self._send_position(self.HOME_JOINTS, "home")
        elif button_idx == self._ready_button:
            self._send_position(self.READY_JOINTS, "ready")
        elif button_idx == self._toggle_debug:
            self._debug_mode = not self._debug_mode
            self.get_logger().info(f"Debug mode: {self._debug_mode}")

    def _send_position(self, joints: List[float], name: str) -> None:
        """Send joint positions directly to servo_controller."""
        msg = Float64MultiArray()
        msg.data = list(joints)
        self._position_pub.publish(msg)
        self.get_logger().info(f"Sent {name} position via servo_controller")

    def _send_gripper(self, position: float, name: str) -> None:
        """Send gripper command to hand_controller."""
        msg = JointTrajectory()
        msg.joint_names = ["left_palm_right_finger"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=0, nanosec=500000000)
        msg.points = [point]
        self._gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper: {name} ({position})")

    def _control_loop(self) -> None:
        """Main control loop."""
        if not self._servo_ready:
            return

        with self._lock:
            if self._latest_axes is None or self._latest_buttons is None:
                return
            axes = list(self._latest_axes)
            buttons = list(self._latest_buttons)

        # Check if L1 is held
        enable_held = (
            self._enable_button < len(buttons) and
            buttons[self._enable_button] == 1
        )

        if not enable_held:
            if self._servo_active:
                self._publish_zero_twist()
                self._servo_active = False
                if self._debug_mode:
                    self.get_logger().info("Servo deactivated")
            return

        if not self._servo_active:
            self._servo_active = True
            if self._debug_mode:
                self.get_logger().info("Servo activated")

        # Sticks → translation + yaw
        vx = self._get_axis(axes, self._axis_x) * self._scale_x * self._linear_scale
        vy = self._get_axis(axes, self._axis_y) * self._scale_y * self._linear_scale
        vz = self._get_axis(axes, self._axis_z) * self._scale_z * self._linear_scale
        wz = self._get_axis(axes, self._axis_yaw) * self._scale_yaw * self._angular_scale

        # D-pad → roll and pitch (dpad_y=roll/X, dpad_x=pitch/Y)
        wx = self._get_axis(axes, self._axis_dpad_y) * self._scale_roll * self._angular_scale
        wy = self._get_axis(axes, self._axis_dpad_x) * self._scale_pitch * self._angular_scale

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self._frame_id
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = wx
        twist.twist.angular.y = wy
        twist.twist.angular.z = wz

        self._twist_pub.publish(twist)

        # L2/R2 triggers → gripper open/close
        l2_val = axes[self._axis_l2] if self._axis_l2 < len(axes) else 1.0
        r2_val = axes[self._axis_r2] if self._axis_r2 < len(axes) else 1.0
        l2_pressed = l2_val < self._trigger_threshold
        r2_pressed = r2_val < self._trigger_threshold

        if l2_pressed and self._gripper_state != "open":
            self._send_gripper(self._gripper_open, "open")
            self._gripper_state = "open"
        elif r2_pressed and self._gripper_state != "closed":
            self._send_gripper(self._gripper_closed, "close")
            self._gripper_state = "closed"
        elif not l2_pressed and not r2_pressed:
            self._gripper_state = None  # reset so next press triggers again

        if self._debug_mode:
            has_lin = abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01
            has_rot = abs(wx) > 0.01 or abs(wy) > 0.01 or abs(wz) > 0.01
            if has_lin or has_rot:
                self.get_logger().info(
                    f"lin:({vx:.2f},{vy:.2f},{vz:.2f}) rot:({wx:.2f},{wy:.2f},{wz:.2f})"
                )

    def _publish_zero_twist(self) -> None:
        """Stop movement."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self._frame_id
        self._twist_pub.publish(twist)

    def _get_axis(self, axes: List[float], idx: int) -> float:
        """Get axis with deadzone."""
        if idx < 0 or idx >= len(axes):
            return 0.0
        value = axes[idx]
        return 0.0 if abs(value) < self._deadzone else value


def main() -> None:
    rclpy.init()
    node = ServoJoystickTeleop()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
