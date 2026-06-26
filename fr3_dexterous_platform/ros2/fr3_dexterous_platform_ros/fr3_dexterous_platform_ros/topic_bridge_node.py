from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TopicBridgeNode(Node):
    """Bridge current FR3 controller topics to stable platform topics."""

    def __init__(self) -> None:
        super().__init__("fr3dex_topic_bridge")
        self.declare_parameter("serl_current_pose_topic", "/NS_1/cartesian_impedance_controller/current_pose")
        self.declare_parameter("serl_command_pose_topic", "/NS_1/cartesian_impedance_controller/equilibrium_pose")
        self.declare_parameter("hand_state_topic", "/cb_right_hand_state")
        self.declare_parameter("hand_command_topic", "/cb_right_hand_control_cmd")

        self.ee_pose_pub = self.create_publisher(PoseStamped, "/franka/ee_pose", 10)
        self.ee_command_pub = self.create_publisher(PoseStamped, "/franka/command/ee_pose", 10)
        self.hand_state_pub = self.create_publisher(JointState, "/linkerhand/right/joint_states", 10)
        self.hand_command_pub = self.create_publisher(JointState, "/linkerhand/right/command", 10)

        self.create_subscription(PoseStamped, self.get_parameter("serl_current_pose_topic").value, self.ee_pose_pub.publish, 10)
        self.create_subscription(PoseStamped, self.get_parameter("serl_command_pose_topic").value, self.ee_command_pub.publish, 10)
        self.create_subscription(JointState, self.get_parameter("hand_state_topic").value, self.hand_state_pub.publish, 10)
        self.create_subscription(JointState, self.get_parameter("hand_command_topic").value, self.hand_command_pub.publish, 10)
        self.get_logger().info("FR3 Dexterous topic bridge ready")


def main() -> None:
    rclpy.init()
    node = TopicBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
