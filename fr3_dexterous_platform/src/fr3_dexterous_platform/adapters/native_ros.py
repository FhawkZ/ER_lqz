"""Native ROS2 FR3 + LinkerHand backend.

This backend talks directly to ROS2 topics and has no dependency on external
robot learning frameworks.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from fr3_dexterous_platform.interfaces import RobotBackend
from fr3_dexterous_platform.schemas import Action, Observation, StampedValue
from fr3_dexterous_platform.utils.quaternion import align_to_previous, normalize
from fr3_dexterous_platform.utils.time import now_s


ARM_POSE_KEYS = ("ee_x", "ee_y", "ee_z", "ori_qx", "ori_qy", "ori_qz", "ori_qw")
HAND_KEYS = ("hand_0", "hand_1", "hand_2", "hand_3", "hand_4", "hand_5")


class NativeRosFr3Backend(RobotBackend):
    def __init__(
        self,
        arm_pose_state_topic: str = "/NS_1/cartesian_impedance_controller/current_pose",
        arm_pose_command_topic: str = "/NS_1/cartesian_impedance_controller/equilibrium_pose",
        hand_state_topic: str = "/cb_right_hand_state",
        hand_command_topic: str = "/cb_right_hand_control_cmd",
        handeye_image_topic: str = "/camera/handeye/color/image_rect_raw",
        fixed_image_topic: str = "/camera/fixed/color/image_rect_raw",
        frame_id: str = "fr3_link0",
        timeout_s: float = 5.0,
    ) -> None:
        self.arm_pose_state_topic = arm_pose_state_topic
        self.arm_pose_command_topic = arm_pose_command_topic
        self.hand_state_topic = hand_state_topic
        self.hand_command_topic = hand_command_topic
        self.handeye_image_topic = handeye_image_topic
        self.fixed_image_topic = fixed_image_topic
        self.frame_id = frame_id
        self.timeout_s = timeout_s

        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._thread = None
        self._owns_rclpy = False
        self._connected = False
        self._arm_pose = None
        self._hand_state = None
        self._handeye_image = None
        self._fixed_image = None
        self._prev_obs_quat: tuple[float, float, float, float] | None = None
        self._seq = 0

    def connect(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import Image, JointState

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        self._node = rclpy.create_node("fr3dex_native_ros_backend")
        self._arm_pub = self._node.create_publisher(PoseStamped, self.arm_pose_command_topic, 10)
        self._hand_pub = self._node.create_publisher(JointState, self.hand_command_topic, 10)
        self._node.create_subscription(PoseStamped, self.arm_pose_state_topic, self._arm_cb, 10)
        self._node.create_subscription(JointState, self.hand_state_topic, self._hand_cb, 10)
        self._node.create_subscription(Image, self.handeye_image_topic, self._handeye_cb, 10)
        self._node.create_subscription(Image, self.fixed_image_topic, self._fixed_cb, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._connected = True

    def disconnect(self) -> None:
        if not self._connected:
            return
        import rclpy

        if self._executor and self._node:
            self._executor.remove_node(self._node)
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._connected = False

    def get_observation(self) -> Observation:
        self._wait_for_required_state()
        receive = now_s()
        with self._lock:
            arm_msg = self._arm_pose
            hand_msg = self._hand_state
            handeye = self._handeye_image
            fixed = self._fixed_image
        assert arm_msg is not None and hand_msg is not None

        p = arm_msg.pose.position
        q_msg = arm_msg.pose.orientation
        q = align_to_previous((q_msg.x, q_msg.y, q_msg.z, q_msg.w), self._prev_obs_quat)
        self._prev_obs_quat = q
        arm_value = {"x": p.x, "y": p.y, "z": p.z, "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]}
        hand_value = {name: float(hand_msg.position[i]) if i < len(hand_msg.position) else 0.0 for i, name in enumerate(HAND_KEYS)}

        values: dict[str, StampedValue] = {
            "arm.ee_pose": StampedValue(arm_value, _stamp_to_s(arm_msg.header.stamp) or receive, receive, self.arm_pose_state_topic, sequence_id=self._seq),
            "hand.joints": StampedValue(hand_value, _stamp_to_s(hand_msg.header.stamp) or receive, receive, self.hand_state_topic, sequence_id=self._seq),
        }
        if handeye is not None:
            values["image.handeye"] = StampedValue(_image_summary(handeye), _stamp_to_s(handeye.header.stamp) or receive, receive, self.handeye_image_topic, sequence_id=self._seq)
        if fixed is not None:
            values["image.fixed"] = StampedValue(_image_summary(fixed), _stamp_to_s(fixed.header.stamp) or receive, receive, self.fixed_image_topic, sequence_id=self._seq)
        self._seq += 1
        return Observation(values=values, receive_time=receive)

    def send_action(self, action: Action) -> Action:
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState

        now_msg = self._node.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now_msg
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(action.values["ee_x.pos"])
        pose.pose.position.y = float(action.values["ee_y.pos"])
        pose.pose.position.z = float(action.values["ee_z.pos"])
        q = normalize(
            (
                float(action.values["ori_qx.pos"]),
                float(action.values["ori_qy.pos"]),
                float(action.values["ori_qz.pos"]),
                float(action.values["ori_qw.pos"]),
            )
        )
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        self._arm_pub.publish(pose)

        hand = JointState()
        hand.header.stamp = now_msg
        hand.name = list(HAND_KEYS)
        hand.position = [float(action.values.get(f"{name}.pos", 0.0)) for name in HAND_KEYS]
        self._hand_pub.publish(hand)
        return Action(values=dict(action.values), source_time=action.source_time, receive_time=now_s(), trace={"backend": "native_ros"})

    def _arm_cb(self, msg: Any) -> None:
        with self._lock:
            self._arm_pose = msg

    def _hand_cb(self, msg: Any) -> None:
        with self._lock:
            self._hand_state = msg

    def _handeye_cb(self, msg: Any) -> None:
        with self._lock:
            self._handeye_image = msg

    def _fixed_cb(self, msg: Any) -> None:
        with self._lock:
            self._fixed_image = msg

    def _wait_for_required_state(self) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._arm_pose is not None and self._hand_state is not None:
                    return
            time.sleep(0.01)
        raise TimeoutError("Timeout waiting for FR3 pose and LinkerHand state")


def _stamp_to_s(stamp: Any) -> float | None:
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _image_summary(msg: Any) -> dict[str, Any]:
    return {
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": msg.encoding,
        "step": int(msg.step),
        "byte_length": len(msg.data),
    }
