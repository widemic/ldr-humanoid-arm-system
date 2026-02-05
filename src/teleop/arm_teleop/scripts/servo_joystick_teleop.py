#!/usr/bin/env python3
"""
MoveIt Servo joystick teleoperation for the LDR humanoid arm.

Publishes TwistStamped commands to MoveIt Servo for real-time Cartesian control.

Controls (DualSense):
  Left stick Y (up/down)    → Move forward/backward (X axis)
  Left stick X (left/right) → Move left/right (Y axis)
  Right stick Y (up/down)   → Move up/down (Z axis)
  Right stick X (left/right) → Rotate around Z (yaw)

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
        self._axis_x = 1          # Left stick Y → robot X
        self._axis_y = 0          # Left stick X → robot Y
        self._axis_z = 3          # Right stick Y → robot Z
        self._axis_yaw = 2        # Right stick X → yaw

        self._scale_x = 1.0
        self._scale_y = -1.0
        self._scale_z = 1.0
        self._scale_yaw = -1.0

        # Button mapping
        self._enable_button = 4   # L1
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

        # Get axis values
        vx = self._get_axis(axes, self._axis_x) * self._scale_x * self._linear_scale
        vy = self._get_axis(axes, self._axis_y) * self._scale_y * self._linear_scale
        vz = self._get_axis(axes, self._axis_z) * self._scale_z * self._linear_scale
        wz = self._get_axis(axes, self._axis_yaw) * self._scale_yaw * self._angular_scale

        # Publish twist
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self._frame_id
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.z = wz

        self._twist_pub.publish(twist)

        if self._debug_mode and (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01 or abs(wz) > 0.01):
            self.get_logger().info(f"Twist: ({vx:.2f}, {vy:.2f}, {vz:.2f}) rot: {wz:.2f}")

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
