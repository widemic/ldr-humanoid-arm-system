#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move)
        self.get_logger().info("Planner running...")

    def move(self):
        msg = Twist()
        msg.linear.x = 0.3
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
