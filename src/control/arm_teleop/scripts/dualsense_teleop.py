#!/usr/bin/env python3
# DualSense teleoperation for a 6-DoF arm using MoveIt Servo.
# Publishes TwistStamped (cartesian jog) and JointJog (joint jog).
# Requires a ROS 2 params file for this node (dualsense.yaml in ROS 2 format).

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Bool
from moveit_msgs.srv import ServoCommandType

class DualSenseTeleop(Node):
    """
    Parameters expected (ROS 2 params format under node name 'dualsense_teleop'):
      axes.lx, axes.ly, axes.rx, axes.ry, axes.l2, axes.r2   (ints)
      buttons.cross, buttons.circle, buttons.square, buttons.triangle,
      buttons.l1, buttons.r1, buttons.share, buttons.options,
      buttons.l3, buttons.r3, buttons.ps, buttons.touch       (ints)
      deadman, mode_toggle, slow_mode, estop_button           (strings: key names from above)
      cartesian.linear, cartesian.angular                     (floats)
      joint.vel                                               (float)
      command_frame                                           (string)
      watchdog_ms                                             (int)
      r2_increases_scale, l2_decreases_scale                  (bool)
    """

    def __init__(self):
        super().__init__('dualsense_teleop')

        # Declare parameters (defaults are safe; override via dualsense.yaml)
        self.declare_parameters('', [
            ('axes.lx', 0), ('axes.ly', 1), ('axes.rx', 3), ('axes.ry', 4),
            ('axes.l2', 2), ('axes.r2', 5),

            ('buttons.cross', 0), ('buttons.circle', 1), ('buttons.square', 2),
            ('buttons.triangle', 3), ('buttons.l1', 4), ('buttons.r1', 5),
            ('buttons.share', 8), ('buttons.options', 9), ('buttons.l3', 10),
            ('buttons.r3', 11), ('buttons.ps', 12), ('buttons.touch', 13),

            ('deadman', 'r1'),
            ('mode_toggle', 'circle'),
            ('slow_mode', 'square'),
            ('estop_button', 'ps'),

            ('cartesian.linear', 0.4),
            ('cartesian.angular', 0.8),
            ('joint.vel', 0.7),
            ('joint.names', [
                'left_shoulder_pitch_rs04_joint',
                'left_shoulder_roll_rs04_joint',
                'left_shoulder_yaw_rs03_joint',
                'left_elbow_rs03_joint',
                'left_wrist_rs02_joint',
                'left_hand_rs02_joint',
            ]),

            ('command_frame', 'base_fixture_link'),
            ('command_type_service', '/moveit_servo/switch_command_type'),
            ('watchdog_ms', 120),
            ('r2_increases_scale', True),
            ('l2_decreases_scale', True),
        ])

        g = self.get_parameter

        # Axes indices
        self.idx = {
            'lx': g('axes.lx').value,
            'ly': g('axes.ly').value,
            'rx': g('axes.rx').value,
            'ry': g('axes.ry').value,
            'l2': g('axes.l2').value,
            'r2': g('axes.r2').value,
        }

        # Button indices
        btn_names = [
            'cross','circle','square','triangle',
            'l1','r1','share','options','l3','r3','ps','touch'
        ]
        self.btn = {name: g(f'buttons.{name}').value for name in btn_names}

        # Controls
        self.deadman_key = g('deadman').value
        self.mode_toggle_key = g('mode_toggle').value
        self.slow_mode_key = g('slow_mode').value
        self.estop_key = g('estop_button').value

        # Scales & frame
        self.lin_max = float(g('cartesian.linear').value)
        self.ang_max = float(g('cartesian.angular').value)
        self.joint_vel = float(g('joint.vel').value)
        joint_names_param = g('joint.names').value
        self.joint_names = [str(name) for name in joint_names_param] if joint_names_param else []
        if not self.joint_names:
            self.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.frame = g('command_frame').value
        self.command_type_service = g('command_type_service').value or '/moveit_servo/switch_command_type'

        # Timing / scaling behavior
        self.watchdog = float(g('watchdog_ms').value) / 1000.0
        self.r2_inc = bool(g('r2_increases_scale').value)
        self.l2_dec = bool(g('l2_decreases_scale').value)

        # Publishers
        self.pub_twist = self.create_publisher(TwistStamped, 'moveit_servo/delta_twist_cmds', 10)
        self.pub_joint = self.create_publisher(JointJog, 'moveit_servo/delta_joint_cmds', 10)
        self.pub_estop = self.create_publisher(Bool, 'arm/estop', 1)

        # Subscriber
        self.sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_cb,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )

        # State
        self.last_msg_time = time.time()
        self.mode = 'cartesian'  # 'cartesian' or 'joint'
        self.prev_toggle_pressed = False

        # Service client for command type switching
        self.switch_cmd_client = self.create_client(ServoCommandType, self.command_type_service)
        self._servo_command_type = None
        self._pending_command_type = None
        self._request_command_type(self.mode)

        # Watchdog timer
        self.create_timer(0.05, self.watchdog_timer)

        self.get_logger().info('DualSense teleop running. Deadman=R1 (default), toggle mode=Circle.')

    def joy_cb(self, msg: Joy):
        self.last_msg_time = time.time()

        buttons = msg.buttons
        axes = msg.axes

        # Button states
        def pressed(name: str) -> bool:
            idx = self.btn[name]
            return 0 <= idx < len(buttons) and buttons[idx] == 1

        deadman_pressed = pressed(self.deadman_key)
        toggle_now = pressed(self.mode_toggle_key)
        estop_pressed = pressed(self.estop_key)
        slow_pressed = pressed(self.slow_mode_key)

        if estop_pressed:
            self.pub_estop.publish(Bool(data=True))
            self.get_logger().warn('E-STOP pressed!')
            return

        # Mode toggle on rising edge
        if toggle_now and not self.prev_toggle_pressed:
            self.mode = 'joint' if self.mode == 'cartesian' else 'cartesian'
            self.get_logger().info(f'Mode switched to: {self.mode}')
            self._request_command_type(self.mode)
        self.prev_toggle_pressed = toggle_now

        # No output unless deadman held
        # if not deadman_pressed:
        #     return

        required_type = self._command_type_for_mode(self.mode)
        if self._servo_command_type != required_type:
            # Ensure Servo has switched before sending
            self._request_command_type(self.mode)
            return

        # Compute dynamic scaling from triggers + slow mode
        r2 = self._safe_axis(axes, self.idx['r2'])
        l2 = self._safe_axis(axes, self.idx['l2'])
        r2n = (r2 + 1.0) * 0.5  # -1..1 → 0..1
        l2n = (l2 + 1.0) * 0.5

        scale = 1.0
        if self.r2_inc:
            scale *= (0.3 + 0.7 * r2n)   # 0.3..1.0
        if self.l2_dec:
            scale *= (1.0 - 0.8 * l2n)   # 1.0..0.2
        if slow_pressed:
            scale *= 0.35

        if self.mode == 'cartesian':
            self._publish_cartesian(axes, scale)
        else:
            self._publish_joint(axes, scale)

    def watchdog_timer(self):
        # If no Joy messages recently, publish zero twist to stop the robot
        if (time.time() - self.last_msg_time) > self.watchdog:
            tw = TwistStamped()
            tw.header.stamp = self.get_clock().now().to_msg()
            tw.header.frame_id = self.frame
            self.pub_twist.publish(tw)

    def _publish_cartesian(self, axes, scale: float):
        lx = self._safe_axis(axes, self.idx['lx'])
        ly = self._safe_axis(axes, self.idx['ly'])
        rx = self._safe_axis(axes, self.idx['rx'])
        ry = self._safe_axis(axes, self.idx['ry'])

        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = self.frame

        # Map: left stick → X/Y, right stick → Z + Yaw
        tw.twist.linear.x  =  self.lin_max * scale * lx
        tw.twist.linear.y  =  self.lin_max * scale * (-ly)
        tw.twist.linear.z  =  self.lin_max * scale * (-ry)
        tw.twist.angular.z =  self.ang_max * scale * rx

        self.pub_twist.publish(tw)

    def _publish_joint(self, axes, scale: float):
        jj = JointJog()
        jj.header.stamp = self.get_clock().now().to_msg()
        jj.header.frame_id = self.frame

        jj.joint_names = self.joint_names
        v = [0.0] * len(self.joint_names)

        # Map the two sticks onto the first four joints if available
        if len(v) > 0:
            v[0] = self.joint_vel * scale * self._safe_axis(axes, self.idx['lx'])
        if len(v) > 1:
            v[1] = self.joint_vel * scale * (-self._safe_axis(axes, self.idx['ly']))
        if len(v) > 2:
            v[2] = self.joint_vel * scale * self._safe_axis(axes, self.idx['rx'])
        if len(v) > 3:
            v[3] = self.joint_vel * scale * (-self._safe_axis(axes, self.idx['ry']))

        jj.velocities = v
        self.pub_joint.publish(jj)

    @staticmethod
    def _safe_axis(axes, idx: int) -> float:
        if idx < 0 or idx >= len(axes):
            return 0.0
        val = axes[idx]
        return float(val) if val is not None else 0.0

    def _command_type_for_mode(self, mode: str) -> int:
        return (
            ServoCommandType.Request.TWIST
            if mode == 'cartesian'
            else ServoCommandType.Request.JOINT_JOG
        )

    def _request_command_type(self, mode: str):
        desired = self._command_type_for_mode(mode)
        if self._pending_command_type == desired or self._servo_command_type == desired:
            return

        if not self.switch_cmd_client.service_is_ready():
            if not self.switch_cmd_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(
                    f'Command type service {self.command_type_service} not available yet.'
                )
                return

        req = ServoCommandType.Request()
        req.command_type = desired

        future = self.switch_cmd_client.call_async(req)
        self._pending_command_type = desired

        def _done_cb(fut):
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Failed to switch Servo command type: {exc}')
                self._pending_command_type = None
                return

            if resp.success:
                self._servo_command_type = desired
                mode_label = 'TWIST' if desired == ServoCommandType.Request.TWIST else 'JOINT_JOG'
                self.get_logger().info(f'Servo command type set to {mode_label}.')
            else:
                self.get_logger().warn('Servo rejected command type switch request.')
            self._pending_command_type = None

        future.add_done_callback(_done_cb)


def main():
    rclpy.init()
    node = DualSenseTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
