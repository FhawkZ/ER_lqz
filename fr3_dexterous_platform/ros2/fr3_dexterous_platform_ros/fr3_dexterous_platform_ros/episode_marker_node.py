from __future__ import annotations

import json
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EpisodeMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("fr3dex_episode_marker")
        self.declare_parameter("task", "pick the red cube and drop it in box")
        self.declare_parameter("recording_id", "")
        self.declare_parameter("operator_id", "unknown")
        self.declare_parameter("environment_id", "default")
        self.recording_id = self.get_parameter("recording_id").value or time.strftime("%Y%m%d_%H%M%S_fr3dex")
        self.task = self.get_parameter("task").value
        self.operator_id = self.get_parameter("operator_id").value
        self.environment_id = self.get_parameter("environment_id").value
        self.event_pub = self.create_publisher(String, "/episode/event", 10)
        self.task_pub = self.create_publisher(String, "/episode/task", 10)
        self.publish_task()
        self.publish_event("start")

    def publish_task(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "recording_id": self.recording_id,
                "task": self.task,
                "operator_id": self.operator_id,
                "environment_id": self.environment_id,
                "source_time": self.get_clock().now().nanoseconds / 1e9,
            },
            ensure_ascii=False,
        )
        self.task_pub.publish(msg)

    def publish_event(self, event: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "recording_id": self.recording_id,
                "event_id": str(uuid.uuid4()),
                "event": event,
                "source_time": self.get_clock().now().nanoseconds / 1e9,
            },
            ensure_ascii=False,
        )
        self.event_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EpisodeMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.publish_event("stop")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
