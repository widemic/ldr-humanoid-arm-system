#!/usr/bin/env python3
"""
Simple DualSense controller node for battery monitoring and special features.

Features:
- Battery level monitoring
- Mute button as emergency stop/brake (with LED indicator)
- Player LEDs (1-4) for mode indication
- Publishes battery status and mute button state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool, UInt8
from pydualsense import pydualsense

class DualSenseNode(Node):
    def __init__(self):
        super().__init__('dualsense_node')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('battery_check_rate', 1.0)  # Hz

        # Publishers
        self.battery_pub = self.create_publisher(Int8, '/dualsense/battery', 10)
        self.mute_pub = self.create_publisher(Bool, '/dualsense/emergency_stop', 10)
        self.mode_pub = self.create_publisher(UInt8, '/dualsense/mode', 10)

        # Initialize controller
        try:
            self.get_logger().info('Looking for DualSense controller...')
            self.ds = pydualsense()

            # Try to find device
            device_infos = self.ds.device_infos()
            if device_infos:
                self.get_logger().info(f'Found {len(device_infos)} DualSense controller(s)')
            else:
                self.get_logger().warn('No DualSense controllers detected')
                self.get_logger().warn('Make sure controller is connected via USB or Bluetooth')
                self.get_logger().warn('You may need to set up udev rules (see README.md)')

            self.ds.init()
            self.get_logger().info('DualSense controller connected successfully')
        except PermissionError as e:
            self.get_logger().error(f'Permission denied accessing controller: {e}')
            self.get_logger().error('Try setting up udev rules (see README.md) or running with sudo')
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to connect to DualSense: {e}')
            self.get_logger().error('Is the controller connected and powered on?')
            raise

        # State variables
        self.current_mode = 1  # Mode 1-4
        self.emergency_stop = False
        self.prev_mute_state = False

        # Set initial LEDs
        self.update_mode_leds()
        self.update_mute_led()

        # Timers
        publish_rate = self.get_parameter('publish_rate').value
        battery_rate = self.get_parameter('battery_check_rate').value

        self.main_timer = self.create_timer(1.0 / publish_rate, self.main_loop)
        self.battery_timer = self.create_timer(1.0 / battery_rate, self.publish_battery)

        self.get_logger().info('DualSense node started')
        self.get_logger().info('Mute button = Emergency Stop')
        self.get_logger().info('PS button short press = Cycle mode (1-4)')

    def main_loop(self):
        """Main control loop - check buttons and publish states."""
        try:
            # Check mute button (toggle emergency stop)
            mute_state = self.ds.state.MicBtn
            if mute_state and not self.prev_mute_state:  # Button pressed
                self.emergency_stop = not self.emergency_stop
                self.update_mute_led()
                self.get_logger().info(f'Emergency stop: {"ACTIVE" if self.emergency_stop else "INACTIVE"}')

                # Publish emergency stop state
                msg = Bool()
                msg.data = self.emergency_stop
                self.mute_pub.publish(msg)

            self.prev_mute_state = mute_state

            # Check PS button (cycle mode)
            # Note: PS button handling varies by connection method
            # This is a simple example - adjust based on your needs
            if self.ds.state.DpadUp:  # Using D-pad Up as mode increment for simplicity
                self.current_mode = (self.current_mode % 4) + 1
                self.update_mode_leds()
                self.get_logger().info(f'Mode changed to: {self.current_mode}')

                # Publish mode
                msg = UInt8()
                msg.data = self.current_mode
                self.mode_pub.publish(msg)

                # Small delay to avoid multiple triggers
                rclpy.spin_once(self, timeout_sec=0.2)

        except Exception as e:
            self.get_logger().error(f'Error in main loop: {e}')

    def publish_battery(self):
        """Publish battery level."""
        try:
            battery_level = self.ds.state.battery

            msg = Int8()
            msg.data = battery_level
            self.battery_pub.publish(msg)

            # Log battery level changes
            if not hasattr(self, 'last_battery_level') or self.last_battery_level != battery_level:
                self.get_logger().info(f'Battery level: {battery_level}%')
                self.last_battery_level = battery_level

        except Exception as e:
            self.get_logger().error(f'Error reading battery: {e}')

    def update_mode_leds(self):
        """Update player LEDs based on current mode (1-4)."""
        try:
            # Set LEDs based on mode (1-4 maps to player ID enum values)
            self.ds.light.setPlayerID(self.current_mode)

        except Exception as e:
            self.get_logger().error(f'Error updating mode LEDs: {e}')

    def update_mute_led(self):
        """Update mute LED based on emergency stop state."""
        try:
            if self.emergency_stop:
                # Mute LED ON (red/orange) when emergency stop active
                self.ds.light.setMicrophoneLED(True)
            else:
                # Mute LED OFF when inactive
                self.ds.light.setMicrophoneLED(False)

        except Exception as e:
            self.get_logger().error(f'Error updating mute LED: {e}')

    def cleanup(self):
        """Clean up controller on shutdown."""
        try:
            # Turn off all LEDs
            self.ds.light.setPlayerID(0)
            self.ds.light.setMicrophoneLED(False)
            self.ds.close()
            self.get_logger().info('DualSense controller disconnected')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DualSenseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
