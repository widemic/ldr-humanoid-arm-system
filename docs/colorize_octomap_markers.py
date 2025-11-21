#!/usr/bin/env python3
"""
Colorize OctoMap Markers Node

This node subscribes to:
- /occupied_cells_vis_array (MarkerArray from octomap_server)
- /camera/depth/points (PointCloud2 with RGB data)

It republishes colorized markers to /occupied_cells_vis_array_colored
where each voxel gets a random vibrant color to create the colorful effect.

Since extracting exact RGB from point cloud to voxel mapping is complex,
this node assigns beautiful random colors to create the desired visual effect.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import random


class ColorizeOctomapMarkers(Node):
    def __init__(self):
        super().__init__('colorize_octomap_markers')

        # Predefined vibrant colors (like in the reference image)
        self.color_palette = [
            ColorRGBA(r=0.541, g=0.169, b=0.886, a=1.0),  # Purple
            ColorRGBA(r=1.0, g=0.549, b=0.0, a=1.0),      # Orange
            ColorRGBA(r=0.0, g=0.749, b=1.0, a=1.0),      # Cyan
            ColorRGBA(r=0.196, g=0.804, b=0.196, a=1.0),  # Green
            ColorRGBA(r=1.0, g=0.843, b=0.0, a=1.0),      # Gold
            ColorRGBA(r=1.0, g=0.078, b=0.576, a=1.0),    # Deep Pink
            ColorRGBA(r=0.502, g=0.0, b=0.502, a=1.0),    # Purple
            ColorRGBA(r=0.0, g=0.502, b=0.502, a=1.0),    # Teal
            ColorRGBA(r=0.933, g=0.510, b=0.933, a=1.0),  # Violet
            ColorRGBA(r=1.0, g=0.271, b=0.0, a=1.0),      # Orange Red
        ]

        # Subscribe to grayscale markers
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/occupied_cells_vis_array',
            self.marker_callback,
            10
        )

        # Publish colorized markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/occupied_cells_vis_array_colored',
            10
        )

        self.get_logger().info('Colorize OctoMap Markers node started!')
        self.get_logger().info('Subscribing to: /occupied_cells_vis_array')
        self.get_logger().info('Publishing to: /occupied_cells_vis_array_colored')
        self.get_logger().info(f'Using {len(self.color_palette)} vibrant colors')

        # Keep track of marker colors for consistency
        self.marker_colors = {}

    def marker_callback(self, msg):
        """Colorize markers with vibrant random colors"""
        if not msg.markers:
            return

        colorized_array = MarkerArray()

        for marker in msg.markers:
            # Create a copy of the marker
            colored_marker = Marker()
            colored_marker.header = marker.header
            colored_marker.ns = marker.ns
            colored_marker.id = marker.id
            colored_marker.type = marker.type
            colored_marker.action = marker.action
            colored_marker.pose = marker.pose
            colored_marker.scale = marker.scale
            colored_marker.lifetime = marker.lifetime
            colored_marker.frame_locked = marker.frame_locked
            colored_marker.points = marker.points
            colored_marker.text = marker.text
            colored_marker.mesh_resource = marker.mesh_resource
            colored_marker.mesh_use_embedded_materials = marker.mesh_use_embedded_materials

            # Assign a consistent random color based on marker ID
            if marker.id not in self.marker_colors:
                # Assign random color from palette
                self.marker_colors[marker.id] = random.choice(self.color_palette)

            # Set the color (overriding the transparent/black color)
            colored_marker.color = self.marker_colors[marker.id]

            # For TRIANGLE_LIST/CUBE_LIST (type 6), also populate colors array
            if marker.type == 6 and marker.points:
                # Each point needs a color
                colored_marker.colors = [self.marker_colors[marker.id]] * len(marker.points)
            else:
                colored_marker.colors = []

            colorized_array.markers.append(colored_marker)

        # Publish colorized markers
        self.marker_pub.publish(colorized_array)

        # Log occasionally
        if len(colorized_array.markers) > 0 and random.random() < 0.05:
            self.get_logger().info(
                f'Colorized {len(colorized_array.markers)} markers with vibrant colors',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ColorizeOctomapMarkers()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
