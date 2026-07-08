#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import math
import threading
import time
from typing import Any, Optional

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from scipy.spatial.transform import Rotation

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotObservation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.quat import normalize_quat_xyzw, prepare_quat_xyzw

from ..robot import Robot
from .config_fr3_eef import FR3EEFConfig

logger = logging.getLogger(__name__)


class FR3EEF(Robot):
    """FR3 end-effector (Cartesian impedance) follower + Linker L6 hand.

    The FR3 arm is driven by publishing target end-effector poses to the SERL
    `cartesian_impedance_controller` (`PoseStamped` on
    `arm_pose_command_topic`). The current EE pose is read back from
    `arm_pose_state_topic`. The Linker L6 hand is controlled at joint level
    (`JointState` on `hand_control_topic`).

    Pose representation in observation / action:
        Observation arm proprioception is selected by ``arm_observation_rep``:
        - ``eef_quat``: ``[x, y, z, qx, qy, qz, qw]`` (legacy redcube checkpoints).
        - ``joint_pos``: 7 measured joint angles (rad), aligned with joint torques.

        Actions remain 7D EE pose + 6D hand joints. Quaternion components are
        sign-aligned frame-to-frame when EE pose is used.

    When ``joint_torque_state_topic`` is set and ``arm_observation_rep=joint_pos``,
    observation also includes 7D filtered external joint torques ``tau_ext_hat_filtered`` (Nm).

    When ``external_wrench_state_topic`` is set, observation includes 6D estimated
    external wrench in the stiffness frame (``K_F_ext_hat_K``: force N + torque Nm).

    EE pose from ``arm_pose_state_topic`` (``geometry_msgs/PoseStamped``) is always
    exposed as ``ee_x`` … ``ori_qw`` in observations, including when
    ``arm_observation_rep=joint_pos`` (async inference can remap state to legacy EE
    checkpoints).
    """

    config_class = FR3EEFConfig
    name = "fr3_eef"

    def __init__(self, config: FR3EEFConfig):
        super().__init__(config)
        self.config = config
        self._node = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._connected = False
        self._owns_rclpy = False
        self._lock = threading.Lock()

        # Cached states.
        self._arm_pose_msg: Optional[PoseStamped] = None
        self._hand_state_msg: Optional[JointState] = None
        self._joint_position_msg: Optional[JointState] = None
        self._joint_torque_msg: Optional[JointState] = None
        self._external_wrench_msg: Optional[WrenchStamped] = None
        self.arm_pose_current = np.zeros(7)  # [x, y, z, qx, qy, qz, qw]
        self.external_wrench_current = np.zeros(len(self.config.ee_wrench_names))
        self.arm_joints_current = np.zeros(len(self.config.arm_joint_names))
        self.hand_joints_current = np.zeros(len(self.config.hand_joint_names))

        # Publishers.
        self._arm_pose_pub = None
        self._hand_pub = None

        # Filter state for the EE pose command.
        self._ema_pos: Optional[np.ndarray] = None
        self._ema_quat: Optional[np.ndarray] = None  # [x, y, z, w]
        # Previous observation quaternion (sign continuity in dataset / policy I/O).
        self._prev_obs_quat: Optional[np.ndarray] = None
        self._cmd_debug_counter = 0

        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _joint_torque_enabled(self) -> bool:
        return bool(self.config.joint_torque_state_topic.strip())

    @property
    def _uses_joint_pos_obs(self) -> bool:
        return self.config.arm_observation_rep == "joint_pos"

    @property
    def _joint_position_enabled(self) -> bool:
        return bool(self.config.joint_position_state_topic.strip())

    @property
    def _external_wrench_enabled(self) -> bool:
        return bool(self.config.external_wrench_state_topic.strip())

    def _validate_config(self) -> None:
        if self.config.arm_observation_rep not in ("eef_quat", "joint_pos"):
            raise ValueError(
                f"arm_observation_rep must be 'eef_quat' or 'joint_pos', "
                f"got {self.config.arm_observation_rep!r}"
            )
        if self._uses_joint_pos_obs and not self._joint_position_enabled:
            raise ValueError(
                "arm_observation_rep='joint_pos' requires joint_position_state_topic"
            )

    # ---------------------------------------------------------------------
    # Feature schemas
    # ---------------------------------------------------------------------
    @property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}
        if self._uses_joint_pos_obs:
            for joint in self.config.arm_joint_names:
                features[f"{joint}.pos"] = float
        for j in self.config.hand_joint_names:
            features[f"{j}.pos"] = float
        for j in self.config.arm_pose_names:
            features[f"{j}.pos"] = float
        if self._external_wrench_enabled:
            for name in self.config.ee_wrench_names[:3]:
                features[f"{name}.force"] = float
            for name in self.config.ee_wrench_names[3:]:
                features[f"{name}.torque"] = float
        if self._uses_joint_pos_obs and self._joint_torque_enabled:
            for joint in self.config.arm_joint_names:
                features[f"{joint}.torque"] = float
        for cam_key in self.cameras:
            cfg = self.config.cameras[cam_key]
            features[cam_key] = (cfg.height, cfg.width, 3)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {f"{j}.pos": float for j in self.config.arm_pose_names}
        features.update({f"{j}.pos": float for j in self.config.hand_joint_names})
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected and all(cam.is_connected for cam in self.cameras.values())

    # ---------------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected")

        self._validate_config()

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        node_name = f"fr3_eef_{self.id or 'default'}"
        self._node = rclpy.create_node(node_name)

        # FR3 EE pose: publish target, subscribe current.
        self._arm_pose_pub = self._node.create_publisher(
            PoseStamped, self.config.arm_pose_command_topic, 10
        )
        self._node.create_subscription(
            PoseStamped, self.config.arm_pose_state_topic, self._arm_pose_cb, 10
        )

        # Linker L6 hand.
        self._hand_pub = self._node.create_publisher(
            JointState, self.config.hand_control_topic, 10
        )
        self._node.create_subscription(
            JointState, self.config.hand_state_topic, self._hand_state_cb, 10
        )

        if self._joint_position_enabled:
            self._node.create_subscription(
                JointState,
                self.config.joint_position_state_topic,
                self._joint_position_cb,
                10,
            )

        if self._joint_torque_enabled:
            self._node.create_subscription(
                JointState,
                self.config.joint_torque_state_topic,
                self._joint_torque_cb,
                10,
            )

        if self._external_wrench_enabled:
            self._node.create_subscription(
                WrenchStamped,
                self.config.external_wrench_state_topic,
                self._external_wrench_cb,
                10,
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._connected = True
        self._cmd_debug_counter = 0
        self._ema_pos = None
        self._ema_quat = None
        self._prev_obs_quat = None

        for cam in self.cameras.values():
            cam.connect()

        hz = float(self.config.control_hz)
        logger.info(
            "%s connected (arm_obs=%s, arm_pose_cmd=%s, arm_pose_state=%s, "
            "joint_position_state=%s, hand_cmd=%s, joint_torque_state=%s, "
            "external_wrench_state=%s, arm_publish=%s, control_hz=%.1f)",
            self,
            self.config.arm_observation_rep,
            self.config.arm_pose_command_topic,
            self.config.arm_pose_state_topic,
            self.config.joint_position_state_topic
            if self._joint_position_enabled
            else "(disabled)",
            self.config.hand_control_topic,
            self.config.joint_torque_state_topic
            if self._joint_torque_enabled
            else "(disabled)",
            self.config.external_wrench_state_topic
            if self._external_wrench_enabled
            else "(disabled)",
            self.config.enable_arm_publish,
            hz,
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s does not require calibration", self)

    def configure(self) -> None:
        pass

    def reset_motion_state(self) -> None:
        """Reset follower-side filter state at episode boundaries."""
        self._ema_pos = None
        self._ema_quat = None
        self._prev_obs_quat = None

    def _align_obs_quat(self, q_raw: np.ndarray) -> np.ndarray:
        """Episode 内 ``q·q_prev>=0``；首帧仅 normalize（与 ROS 同支）。"""
        q = prepare_quat_xyzw(q_raw, q_prev=self._prev_obs_quat)
        self._prev_obs_quat = q.copy()
        return q

    # ---------------------------------------------------------------------
    # ROS callbacks
    # ---------------------------------------------------------------------
    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        q_raw = normalize_quat_xyzw(
            np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        )
        with self._lock:
            self._arm_pose_msg = msg
            q_cont = self._align_obs_quat(q_raw)
            self.arm_pose_current = np.array(
                [p.x, p.y, p.z, q_cont[0], q_cont[1], q_cont[2], q_cont[3]],
                dtype=np.float64,
            )

    def _hand_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._hand_state_msg = msg
            self.hand_joints_current = np.array(msg.position, dtype=np.float64)

    def _joint_position_cb(self, msg: JointState) -> None:
        with self._lock:
            self._joint_position_msg = msg

    def _joint_torque_cb(self, msg: JointState) -> None:
        with self._lock:
            self._joint_torque_msg = msg

    def _external_wrench_cb(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        t = msg.wrench.torque
        with self._lock:
            self._external_wrench_msg = msg
            self.external_wrench_current = np.array(
                [f.x, f.y, f.z, t.x, t.y, t.z],
                dtype=np.float64,
            )

    def _ordered_joint_torques(self) -> list[float]:
        with self._lock:
            msg = self._joint_torque_msg
        if msg is None:
            return [0.0] * len(self.config.arm_joint_names)

        if not msg.effort:
            return [0.0] * len(self.config.arm_joint_names)

        if msg.name and len(msg.name) == len(msg.effort):
            name_to_effort = {name: effort for name, effort in zip(msg.name, msg.effort)}
            return [
                float(name_to_effort.get(joint, 0.0)) for joint in self.config.arm_joint_names
            ]

        if len(msg.effort) < len(self.config.arm_joint_names):
            efforts = [float(effort) for effort in msg.effort[: len(self.config.arm_joint_names)]]
            while len(efforts) < len(self.config.arm_joint_names):
                efforts.append(0.0)
            return efforts

        return [float(effort) for effort in msg.effort[: len(self.config.arm_joint_names)]]

    def _ordered_joint_positions(self) -> list[float]:
        with self._lock:
            msg = self._joint_position_msg
        if msg is None:
            return [0.0] * len(self.config.arm_joint_names)

        if not msg.position:
            return [0.0] * len(self.config.arm_joint_names)

        if msg.name and len(msg.name) == len(msg.position):
            name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            return [
                float(name_to_pos.get(joint, 0.0)) for joint in self.config.arm_joint_names
            ]

        if len(msg.position) < len(self.config.arm_joint_names):
            positions = [float(pos) for pos in msg.position[: len(self.config.arm_joint_names)]]
            while len(positions) < len(self.config.arm_joint_names):
                positions.append(0.0)
            return positions

        return [float(pos) for pos in msg.position[: len(self.config.arm_joint_names)]]

    def _wait_for_state(self) -> None:
        deadline = time.monotonic() + self.config.timeout_s
        arm_ok = False
        hand_ok = False
        joint_pos_ok = (not self._uses_joint_pos_obs) or not self._joint_position_enabled
        torque_ok = (not self._uses_joint_pos_obs) or (not self._joint_torque_enabled)
        wrench_ok = not self._external_wrench_enabled
        while time.monotonic() < deadline:
            with self._lock:
                arm_ok = self._arm_pose_msg is not None
                hand_ok = self._hand_state_msg is not None
                joint_pos_ok = (
                    (not self._uses_joint_pos_obs)
                    or (not self._joint_position_enabled)
                    or self._joint_position_msg is not None
                )
                torque_ok = (
                    (not self._uses_joint_pos_obs)
                    or (not self._joint_torque_enabled)
                    or self._joint_torque_msg is not None
                )
                wrench_ok = (
                    (not self._external_wrench_enabled)
                    or self._external_wrench_msg is not None
                )
            if arm_ok and hand_ok and joint_pos_ok and torque_ok and wrench_ok:
                return
            time.sleep(0.01)
        if not arm_ok:
            missing = "arm_pose"
        elif not hand_ok:
            missing = "hand"
        elif not joint_pos_ok:
            missing = "joint_position"
        elif not torque_ok:
            missing = "joint_torque"
        else:
            missing = "external_wrench"
        raise TimeoutError(f"Timeout waiting for {missing} state on ROS2 topics")

    def _fill_eef_pose_obs(self, obs: dict[str, Any], arm_pose: np.ndarray) -> None:
        for i, j in enumerate(self.config.arm_pose_names):
            obs[f"{j}.pos"] = float(arm_pose[i])

    def _fill_external_wrench_obs(self, obs: dict[str, Any], wrench: np.ndarray) -> None:
        for i, name in enumerate(self.config.ee_wrench_names[:3]):
            obs[f"{name}.force"] = float(wrench[i])
        for i, name in enumerate(self.config.ee_wrench_names[3:], start=3):
            obs[f"{name}.torque"] = float(wrench[i])

    def _fill_joint_pos_obs(self, obs: dict[str, Any], joint_positions: list[float]) -> None:
        for idx, joint in enumerate(self.config.arm_joint_names):
            obs[f"{joint}.pos"] = joint_positions[idx]

    # ---------------------------------------------------------------------
    # Observation
    # ---------------------------------------------------------------------
    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self._wait_for_state()
        with self._lock:
            arm_pose = self.arm_pose_current.copy()
            hand_pos = self.hand_joints_current.copy()
            external_wrench = self.external_wrench_current.copy()
        joint_positions = self._ordered_joint_positions()
        joint_torques = self._ordered_joint_torques()

        obs: dict[str, Any] = {}
        if self._uses_joint_pos_obs:
            self._fill_joint_pos_obs(obs, joint_positions)
        self._fill_eef_pose_obs(obs, arm_pose)
        if self._external_wrench_enabled:
            self._fill_external_wrench_obs(obs, external_wrench)
        for i, j in enumerate(self.config.hand_joint_names):
            # Fall back to 0.0 if the hand publisher sends fewer joints than expected.
            obs[f"{j}.pos"] = float(hand_pos[i]) if i < len(hand_pos) else 0.0
        if self._uses_joint_pos_obs and self._joint_torque_enabled:
            for idx, joint in enumerate(self.config.arm_joint_names):
                obs[f"{joint}.torque"] = joint_torques[idx]
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    # ---------------------------------------------------------------------
    # Action
    # ---------------------------------------------------------------------
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        # ---- 1. Parse arm pose action ----
        raw_pose = []
        for j in self.config.arm_pose_names:
            key = f"{j}.pos"
            if key not in action:
                raise ValueError(f"Missing action key: {key}")
            value = float(action[key])
            if not math.isfinite(value):
                logger.warning("Received non-finite action for %s, ignoring this frame.", key)
                return action
            raw_pose.append(value)

        target_pos = np.array(raw_pose[:3], dtype=np.float64)
        target_quat = normalize_quat_xyzw(np.array(raw_pose[3:7], dtype=np.float64))

        # ---- 2. Hand action ----
        hand_pos = []
        for j in self.config.hand_joint_names:
            key = f"{j}.pos"
            if key not in action:
                raise ValueError(f"Missing action key: {key}")
            hand_pos.append(float(action[key]))

        # ---- 3. Hand-only mode (skip arm pose publish) ----
        if not self.config.enable_arm_publish:
            self._publish_hand(hand_pos)
            return action

        # ---- 4. Need a measured pose to seed the EMA on the very first frame ----
        with self._lock:
            current_pose_msg = self._arm_pose_msg

        if current_pose_msg is None:
            logger.warning("No EE pose received yet. Skip this arm command frame.")
            return action

        meas_pos = np.array(
            [
                current_pose_msg.pose.position.x,
                current_pose_msg.pose.position.y,
                current_pose_msg.pose.position.z,
            ],
            dtype=np.float64,
        )
        meas_quat = np.array(
            [
                current_pose_msg.pose.orientation.x,
                current_pose_msg.pose.orientation.y,
                current_pose_msg.pose.orientation.z,
                current_pose_msg.pose.orientation.w,
            ],
            dtype=np.float64,
        )

        # ---- 5. EMA filtering ----
        # Position: standard EMA per axis.
        alpha_pos = max(0.01, min(1.0, float(self.config.ema_alpha_pos)))
        if self._ema_pos is None:
            self._ema_pos = meas_pos.copy()
        safe_pos = alpha_pos * target_pos + (1.0 - alpha_pos) * self._ema_pos
        self._ema_pos = safe_pos

        # Orientation: SLERP between previous filtered quat and target quat.
        alpha_rot = max(0.01, min(1.0, float(self.config.ema_alpha_rot)))
        if self._ema_quat is None:
            self._ema_quat = normalize_quat_xyzw(meas_quat)
        target_quat = prepare_quat_xyzw(target_quat, q_prev=self._ema_quat)
        safe_quat = self._slerp(self._ema_quat, target_quat, alpha_rot)
        # Normalize to guard against numerical drift.
        norm = np.linalg.norm(safe_quat)
        if norm > 1e-9:
            safe_quat = safe_quat / norm
        self._ema_quat = normalize_quat_xyzw(safe_quat)

        # ---- 6. Publish PoseStamped ----
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.config.arm_pose_frame_id
        pose_msg.pose.position.x = float(safe_pos[0])
        pose_msg.pose.position.y = float(safe_pos[1])
        pose_msg.pose.position.z = float(safe_pos[2])
        pose_msg.pose.orientation.x = float(safe_quat[0])
        pose_msg.pose.orientation.y = float(safe_quat[1])
        pose_msg.pose.orientation.z = float(safe_quat[2])
        pose_msg.pose.orientation.w = float(safe_quat[3])
        self._arm_pose_pub.publish(pose_msg)
        self._cmd_debug_counter += 1

        # ---- 7. Publish hand command ----
        self._publish_hand(hand_pos)

        return action

    # ---------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------
    def _publish_hand(self, hand_pos: list[float]) -> None:
        hand_msg = JointState()
        hand_msg.header.stamp = self._node.get_clock().now().to_msg()
        hand_msg.name = list(self.config.hand_joint_names)
        hand_msg.position = hand_pos
        self._hand_pub.publish(hand_msg)

    @staticmethod
    def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions ``q0`` and ``q1``.

        Both quaternions use scipy's ``[x, y, z, w]`` ordering.
        """
        dot = float(np.dot(q0, q1))
        # If quaternions are very close, fall back to linear interpolation.
        if dot > 0.9995:
            out = q0 + t * (q1 - q0)
            return out
        dot = max(-1.0, min(1.0, dot))
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        if sin_theta_0 < 1e-9:
            return q0.copy()
        theta = theta_0 * t
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        return s0 * q0 + s1 * q1

    # ---------------------------------------------------------------------
    # Disconnect
    # ---------------------------------------------------------------------
    def disconnect(self) -> None:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()

        if self._executor and self._node:
            self._executor.remove_node(self._node)
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

        self._connected = False
        self._executor = None
        self._node = None
        self._spin_thread = None
        logger.info("%s disconnected", self)
