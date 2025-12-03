#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class SimpleObjectTracker(Node):
    """
    Simple object tracker that detects the closest object in the point cloud.
    """

    def __init__(self):
        super().__init__('simple_object_tracker')

        # Subscribe to point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # Publish detected object position
        self.object_pub = self.create_publisher(Point, '/tracked_object/position', 10)
        self.status_pub = self.create_publisher(String, '/tracked_object/status', 10)

        # Parameters
        self.min_z = 0.1  # Minimum distance (m)
        self.max_z = 2.0  # Maximum distance (m)
        self.min_points = 50  # Minimum points to consider valid object

        self.get_logger().info('Simple Object Tracker started')

    def pointcloud_callback(self, msg):
        """Process point cloud and find closest object."""
        try:
            # Convert point cloud to numpy array
            points = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                x, y, z = point
                # Filter by distance
                if self.min_z < z < self.max_z:
                    points.append([x, y, z])

            if len(points) < self.min_points:
                status_msg = String()
                status_msg.data = "No object detected"
                self.status_pub.publish(status_msg)
                return

            points = np.array(points)

            # Find closest point
            distances = np.linalg.norm(points, axis=1)
            closest_idx = np.argmin(distances)
            closest_point = points[closest_idx]

            # Publish object position
            object_pos = Point()
            object_pos.x = float(closest_point[0])
            object_pos.y = float(closest_point[1])
            object_pos.z = float(closest_point[2])
            self.object_pub.publish(object_pos)

            # Publish status
            status_msg = String()
            status_msg.data = f"Object at ({object_pos.x:.2f}, {object_pos.y:.2f}, {object_pos.z:.2f})"
            self.status_pub.publish(status_msg)

            self.get_logger().info(status_msg.data, throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    tracker = SimpleObjectTracker()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
