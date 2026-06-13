from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOPICS = [
    "/franka/ee_pose",
    "/franka/command/ee_pose",
    "/linkerhand/right/joint_states",
    "/linkerhand/right/command",
    "/camera/handeye/color/image_rect_raw",
    "/camera/handeye/depth/image_rect_raw",
    "/camera/handeye/metadata",
    "/camera/fixed/color/image_rect_raw",
    "/camera/fixed/depth/image_rect_raw",
    "/camera/fixed/metadata",
    "/teleop/raw",
    "/teleop/retargeted_action",
    "/policy/client_observation",
    "/policy/server_response",
    "/policy/action",
    "/policy/action_filtered",
    "/episode/event",
    "/episode/task",
]


class RecordingManifestNode(Node):
    def __init__(self) -> None:
        super().__init__("fr3dex_recording_manifest")
        self.declare_parameter("recording_id", "manual")
        self.declare_parameter("task", "pick the red cube and drop it in box")
        self.declare_parameter("output_dir", "/tmp/fr3dex_records")
        self.pub = self.create_publisher(String, "/recording/manifest", 10)
        self.timer = self.create_timer(2.0, self.publish_manifest)
        self.publish_manifest()

    def publish_manifest(self) -> None:
        recording_id = self.get_parameter("recording_id").value
        output_dir = self.get_parameter("output_dir").value
        msg = String()
        msg.data = json.dumps(
            {
                "recording_id": recording_id,
                "task": self.get_parameter("task").value,
                "output_dir": output_dir,
                "topics": TOPICS,
                "rosbag2_command": ["ros2", "bag", "record", "--storage", "mcap", "--output", f"{output_dir}/{recording_id}", *TOPICS],
            },
            ensure_ascii=False,
        )
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RecordingManifestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
