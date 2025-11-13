#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy


class JoyInspector(Node):
    def __init__(self):
        super().__init__('joy_inspector')
        self.sub = self.create_subscription(
            Joy,
            'joy',
            self.cb,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.get_logger().info('Joy inspector started. Move sticks / press buttons to see indices.')

    def cb(self, msg: Joy):
        self.get_logger().info(f"axes: {[round(a,3) for a in msg.axes]}\nbuttons: {msg.buttons}")

def main():
    rclpy.init()
    node = JoyInspector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
