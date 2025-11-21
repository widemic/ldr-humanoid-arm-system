#!/usr/bin/env python3
"""
Mirror Gazebo models into the MoveIt planning scene as simple geometric shapes.

This is a light-weight adaptation of the Addison mycobot demo: we watch Gazebo
model states and publish CollisionObjects (boxes/cylinders) into MoveIt so that
planning respects the obstacles you see in simulation.

Usage:
  ros2 run arm_moveit_config gazebo_shape_sync.py

Key parameters:
  - frame_id: planning frame to anchor collision objects (defaults to base_link)
  - model_shapes_yaml: YAML dictionary mapping model names -> shape definitions
        model_shapes_yaml: |
          box: {type: box, size: [0.3, 0.3, 0.3]}
          can: {type: cylinder, radius: 0.03, height: 0.13}
  - model_states_topic: gazebo_msgs/ModelStates topic to watch (default: /model_states)

Notes:
  - Requires moveit_commander (MoveIt 2 Python bindings) and gazebo_msgs (bridged from gz).
  - The model name must match exactly the Gazebo model name to be mirrored.
"""

from typing import Dict

import geometry_msgs.msg
import rclpy
from gazebo_msgs.msg import ModelStates
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from rclpy.node import Node


def _default_shapes() -> Dict[str, dict]:
    # Default demo shapes; override via parameters to match your Gazebo models.
    return {
        "box": {"type": "box", "size": [0.3, 0.3, 0.3]},
        "cylinder": {"type": "cylinder", "radius": 0.04, "height": 0.14},
    }


class GazeboShapeSync(Node):
    def __init__(self):
        super().__init__("gazebo_shape_sync")
        roscpp_initialize([])

        self.frame_id = (
            self.declare_parameter("frame_id", "base_link")
            .get_parameter_value()
            .string_value
        )
        shapes_yaml = (
            self.declare_parameter("model_shapes_yaml", "")
            .get_parameter_value()
            .string_value
        )
        self.model_shapes = self._parse_shapes(shapes_yaml)
        self.model_states_topic = (
            self.declare_parameter("model_states_topic", "/model_states")
            .get_parameter_value()
            .string_value
        )

        self.scene = PlanningSceneInterface(synchronous=True)
        self._last_seen = {}

        self.create_subscription(
            ModelStates, self.model_states_topic, self._states_cb, 10
        )

        self.get_logger().info(
            f"Mirroring Gazebo models into MoveIt planning scene on {self.model_states_topic}"
        )
        self.get_logger().info(
            f"Tracking models: {list(self.model_shapes.keys())} (frame_id={self.frame_id})"
        )

        # Periodic cleanup of stale models
        self.create_timer(5.0, self._cleanup_stale)

    def _parse_shapes(self, shapes_yaml: str) -> Dict[str, dict]:
        if not shapes_yaml:
            return _default_shapes()
        try:
            import yaml

            parsed = yaml.safe_load(shapes_yaml) or {}
            if not isinstance(parsed, dict):
                raise ValueError("model_shapes_yaml must decode to a dict")
            return parsed
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to parse model_shapes_yaml, falling back to defaults: {exc}"
            )
            return _default_shapes()

    def _states_cb(self, msg: ModelStates):
        names = msg.name
        poses = msg.pose

        # Build a quick lookup for current models we care about
        present = set()
        for name, pose in zip(names, poses):
            if name not in self.model_shapes:
                continue
            present.add(name)
            self._upsert_collision(name, pose, self.model_shapes[name])

        self._remove_missing(present)

    def _upsert_collision(self, name: str, pose: geometry_msgs.msg.Pose, shape: dict):
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        shape_type = shape.get("type", "")
        if shape_type == "box":
            size = shape.get("size", [0.1, 0.1, 0.1])
            self.scene.add_box(name, ps, size=size)
        elif shape_type == "cylinder":
            radius = float(shape.get("radius", 0.05))
            height = float(shape.get("height", 0.1))
            self.scene.add_cylinder(name, height=height, radius=radius, pose=ps)
        else:
            self.get_logger().warn(
                f"Unsupported shape for model '{name}': {shape_type}"
            )
            return

        self._last_seen[name] = self.get_clock().now()
        self.get_logger().debug(f"Updated collision object '{name}' as {shape_type}")

    def _remove_missing(self, present: set):
        # If a tracked model disappears from Gazebo, remove its collision object
        missing = [name for name in self._last_seen.keys() if name not in present]
        if not missing:
            return
        self.scene.remove_world_object(name=missing)
        for name in missing:
            self._last_seen.pop(name, None)
            self.get_logger().info(f"Removed collision object for missing model '{name}'")

    def _cleanup_stale(self):
        # Remove any model that hasn't been seen in 10 seconds
        now = self.get_clock().now()
        stale = [
            name
            for name, timestamp in self._last_seen.items()
            if (now - timestamp).nanoseconds > 10_000_000_000
        ]
        if stale:
            self.scene.remove_world_object(name=stale)
            for name in stale:
                self._last_seen.pop(name, None)
                self.get_logger().info(f"Removed stale collision object '{name}'")


def main():
    rclpy.init()
    node = GazeboShapeSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        roscpp_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
