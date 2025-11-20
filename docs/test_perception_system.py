#!/usr/bin/env python3
"""
Quick diagnostic script to test perception system setup
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PerceptionDiagnostic(Node):
    def __init__(self):
        super().__init__('perception_diagnostic')

        self.get_logger().info('Perception Diagnostic Node Started')

        # Subscribe to camera point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        self.frame_count = 0

    def pointcloud_callback(self, msg):
        """Check if we're receiving point cloud data"""
        self.frame_count += 1

        # Count points
        point_count = 0
        for _ in pc2.read_points(msg, skip_nans=True):
            point_count += 1
            if point_count > 100:  # Just sample first 100
                break

        self.get_logger().info(
            f'Frame {self.frame_count}: Received point cloud from "{msg.header.frame_id}" '
            f'with {msg.width * msg.height} total points (sampled {point_count}+ valid points)'
        )


def main():
    rclpy.init()
    node = PerceptionDiagnostic()

    print("\n" + "="*60)
    print("PERCEPTION SYSTEM DIAGNOSTIC")
    print("="*60)
    print("\nWaiting for point cloud data on /camera/depth/points...")
    print("Press Ctrl+C to exit\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nDiagnostic complete!")
        print(f"Total frames received: {node.frame_count}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
