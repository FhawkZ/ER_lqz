"""Native mocap-to-FR3 action source.

Use mocap wrist pose deltas to update an absolute FR3 end-effector target.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from fr3_dexterous_platform.interfaces import ActionSource
from fr3_dexterous_platform.schemas import Action, Observation
from fr3_dexterous_platform.teleop.mocap_delta import DeltaMocapRetargeter, MocapPose
from fr3_dexterous_platform.utils.time import now_s


class NativeMocapRetargetSource(ActionSource):
    """ROS2 mocap pose subscriber -> FR3 EE + LinkerHand action.

    Expected mocap input:
    - `mocap_pose_topic`: `geometry_msgs/PoseStamped` for the right hand/wrist.
    - optional `hand_command_topic`: `sensor_msgs/JointState` with 6 LinkerHand
      command positions. If absent, hand command defaults to open.
    """

    def __init__(
        self,
        mocap_pose_topic: str = "/mocap/right_hand/pose",
        hand_command_topic: str = "/mocap/right_hand/hand_cmd",
        delta_pos_gain: float = 1.4,
        delta_rot_gain: float = 1.0,
        max_delta_pos_per_cycle: float = 0.04,
        max_delta_rot_per_cycle: float = 0.30,
        timeout_s: float = 5.0,
    ) -> None:
        self.mocap_pose_topic = mocap_pose_topic
        self.hand_command_topic = hand_command_topic
        self.retargeter = DeltaMocapRetargeter(
            delta_pos_gain=delta_pos_gain,
            delta_rot_gain=delta_rot_gain,
            max_delta_pos_per_cycle=max_delta_pos_per_cycle,
            max_delta_rot_per_cycle=max_delta_rot_per_cycle,
        )
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._thread = None
        self._owns_rclpy = False
        self._connected = False
        self._latest_mocap_pose = None
        self._latest_hand_cmd = None
        self._prev_mocap_pose = None

    def connect(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        self._node = rclpy.create_node("fr3dex_native_mocap_source")
        self._node.create_subscription(PoseStamped, self.mocap_pose_topic, self._mocap_cb, 10)
        self._node.create_subscription(JointState, self.hand_command_topic, self._hand_cb, 10)
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

    def reset(self) -> None:
        with self._lock:
            self._prev_mocap_pose = self._latest_mocap_pose
            self.retargeter.reset()

    def get_action(self, observation: Observation) -> Action:
        self._wait_for_mocap()
        t = now_s()
        with self._lock:
            current_pose = self._latest_mocap_pose
            hand_cmd = self._latest_hand_cmd
        assert current_pose is not None

        pos, quat = _pose_to_tuple(current_pose)
        pose = MocapPose(position=pos, quaternion_xyzw=quat, source_time=t)
        values = self.retargeter.action_values(observation, pose, _hand_cmd_values(hand_cmd))
        return Action(values=values, source_time=t, receive_time=now_s(), trace={"source": "native_mocap"})

    def _mocap_cb(self, msg: Any) -> None:
        with self._lock:
            self._latest_mocap_pose = msg

    def _hand_cb(self, msg: Any) -> None:
        with self._lock:
            self._latest_hand_cmd = msg

    def _wait_for_mocap(self) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_mocap_pose is not None:
                    return
            time.sleep(0.01)
        raise TimeoutError(f"Timeout waiting for mocap pose on {self.mocap_pose_topic}")


def _pose_to_tuple(msg: Any) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    p = msg.pose.position
    q = msg.pose.orientation
    return (float(p.x), float(p.y), float(p.z)), (float(q.x), float(q.y), float(q.z), float(q.w))


def _hand_cmd_values(msg: Any | None) -> list[float]:
    if msg is None:
        return [255.0] * 6
    values = list(getattr(msg, "position", []))
    return [float(values[i]) if i < len(values) else 255.0 for i in range(6)]
