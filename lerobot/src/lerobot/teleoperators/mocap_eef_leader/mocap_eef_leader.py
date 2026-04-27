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

"""MocapEefLeader.

Mocap-driven leader for the ``fr3_eef`` follower. Unlike ``MocapLeader``
this implementation does **not** run any inverse kinematics: it accumulates
the mocap RightHand pose deltas into an absolute end-effector pose target
in the FR3 base frame and emits that pose as ``[ee_x, ee_y, ee_z, ori_r,
ori_p, ori_y]`` (extrinsic xyz Euler), which is exactly the schema
consumed by :class:`FR3EEF.send_action`.

Coordinate / rotation conventions (must match ``fr3_eef``)::

    Position : meters, in the FR3 base link frame (``fr3_link0`` by default)
    Orientation : extrinsic xyz Euler angles (radians), produced via
                  ``Rotation.as_euler("xyz")``.
    Mocap → FR3 axis mapping (same as ``MocapLeader``)::
        X_robot =  Y_mocap
        Y_robot = -X_mocap
        Z_robot =  Z_mocap
    Rotation accumulation::
        R_new = exp(delta_rotvec_in_base) @ R_current
    which is the same convention as ``MocapLeader``'s IK target
    (``target_r = pin.exp3(delta) @ R0``).
"""

import logging
import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_mocap_eef_leader import MocapEefLeaderConfig

logger = logging.getLogger(__name__)


try:
    from lerobot.third_party.mocap_ros_py.mocap_robotapi import (
        MCPApplication,
        MCPAvatar,
        MCPEventType,
        MCPSettings,
    )
except ImportError:
    try:
        from ..mocap_leader.mocap_robotapi import (  # type: ignore[no-redef]
            MCPApplication,
            MCPAvatar,
            MCPEventType,
            MCPSettings,
        )
    except ImportError as exc:  # pragma: no cover - depends on local install
        MCPApplication = MCPAvatar = MCPEventType = MCPSettings = None  # type: ignore[assignment]
        logger.warning(
            "mocap_robotapi not available: MocapEefLeader will not work without it (%s)",
            exc,
        )


# ---------------------------------------------------------------------------
# Mocap helpers (kept in sync with `mocap_leader.mocap_leader`).
# Duplicated here so this module does not pull in pinocchio at import time.
# ---------------------------------------------------------------------------

links_parent = {
    "Hips": "world",
    "Spine": "Hips",
    "Spine1": "Spine",
    "Spine2": "Spine1",
    "RightShoulder": "Spine2",
    "RightArm": "RightShoulder",
    "RightForeArm": "RightArm",
    "RightHand": "RightForeArm",
    "RightHandThumb1": "RightHand",
    "RightHandThumb2": "RightHandThumb1",
    "RightHandThumb3": "RightHandThumb2",
    "RightInHandIndex": "RightHand",
    "RightHandIndex1": "RightInHandIndex",
    "RightHandIndex2": "RightHandIndex1",
    "RightHandIndex3": "RightHandIndex2",
    "RightInHandMiddle": "RightHand",
    "RightHandMiddle1": "RightInHandMiddle",
    "RightHandMiddle2": "RightHandMiddle1",
    "RightHandMiddle3": "RightHandMiddle2",
    "RightInHandRing": "RightHand",
    "RightHandRing1": "RightInHandRing",
    "RightHandRing2": "RightHandRing1",
    "RightHandRing3": "RightHandRing2",
    "RightInHandPinky": "RightHand",
    "RightHandPinky1": "RightInHandPinky",
    "RightHandPinky2": "RightHandPinky1",
    "RightHandPinky3": "RightHandPinky2",
}


def axis_to_ros_position(position):
    """PN-Studio (cm) -> ROS-style (m): (x_pn, y_pn, z_pn) -> (z/100, x/100, y/100)."""
    return (position[2] / 100.0, position[0] / 100.0, position[1] / 100.0)


def axis_to_ros_quaternion(rotation):
    """PN-Studio quaternion (w,x,y,z) -> ROS-style (qx, qy, qz, qw) using the
    same axis remap as ``axis_to_ros_position``: outputs ``(z, x, y, w)``.
    """
    return (rotation[3], rotation[1], rotation[2], rotation[0])


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[:3]


def compose_transform(parent_pos, parent_quat, local_pos, local_quat):
    rotated = quat_rotate(parent_quat, local_pos)
    return (
        (
            parent_pos[0] + rotated[0],
            parent_pos[1] + rotated[1],
            parent_pos[2] + rotated[2],
        ),
        quat_multiply(parent_quat, local_quat),
    )


# Hand finger-bend extraction (identical to mocap_leader).
FINGER_BASE_ANGLE_JOINTS = [
    ("RightHandIndex3", "RightHandIndex1", "RightInHandIndex", "RightHandIndex2"),
    ("RightHandMiddle3", "RightHandMiddle1", "RightInHandMiddle", "RightHandMiddle2"),
    ("RightHandRing3", "RightHandRing1", "RightInHandRing", "RightHandRing2"),
    ("RightHandPinky3", "RightHandPinky1", "RightInHandPinky", "RightHandPinky2"),
    ("RightHandThumb3", "RightHandThumb2", "RightHandThumb1", "RightHandThumb3"),
]
ANGLE_OPEN = math.pi
ANGLE_CLOSED = 1.55


def _calculate_plane_angle(plane1_points, plane2_points) -> float:
    v1_1 = plane1_points[1] - plane1_points[0]
    v1_2 = plane1_points[2] - plane1_points[0]
    normal1 = np.cross(v1_1, v1_2)
    normal1 = normal1 / (np.linalg.norm(normal1) + 1e-10)

    v2_1 = plane2_points[1] - plane2_points[0]
    v2_2 = plane2_points[2] - plane2_points[0]
    normal2 = np.cross(v2_1, v2_2)
    normal2 = normal2 / (np.linalg.norm(normal2) + 1e-10)

    cos_angle = np.clip(float(np.dot(normal1, normal2)), -1.0, 1.0)
    angle = math.acos(cos_angle)
    return min(angle, math.pi - angle)


def build_local_transforms(avatar) -> dict:
    """Local transforms with axis_to_ros conversion (meters), used for FK of
    the right hand pose."""
    transforms = {}
    for joint in avatar.get_joints():
        name = joint.get_name()
        lp = joint.get_local_position()
        if lp is None:
            lp = (0.0, 0.0, 0.0)
        position = axis_to_ros_position(lp)
        rotation = axis_to_ros_quaternion(joint.get_local_rotation())
        transforms[name] = (position, rotation)
    return transforms


def _build_local_transforms(avatar) -> dict:
    """Raw local transforms (cm + (w,x,y,z)) used for hand finger geometry."""
    local_transforms = {}
    for joint in avatar.get_joints():
        name = joint.get_name()
        lp = joint.get_local_position()
        if lp is None:
            lp = (0.0, 0.0, 0.0)
        lr = joint.get_local_rotation()
        local_transforms[name] = (lp, lr)
    return local_transforms


def get_global_transform(name, local_transforms, cache):
    if name in cache:
        return cache[name]
    if name not in local_transforms:
        return None
    parent = links_parent.get(name, "world")
    if parent == "world":
        cache[name] = local_transforms[name]
        return cache[name]
    parent_tf = get_global_transform(parent, local_transforms, cache)
    if parent_tf is None:
        return None
    cache[name] = compose_transform(
        parent_tf[0], parent_tf[1], local_transforms[name][0], local_transforms[name][1]
    )
    return cache[name]


def _build_right_hand_relative_transforms(avatar, local_transforms: dict) -> dict:
    """All RightHand-subtree joints expressed relative to RightHand."""
    children: dict[str, list[str]] = {}
    parent_of: dict[str, str] = {}
    for joint in avatar.get_joints():
        child_name = joint.get_name()
        tag = joint.get_tag()
        try:
            parent_tag = joint.get_parent_joint_tag(tag)
        except RuntimeError:
            parent_tag = -1
        if parent_tag < 0 or parent_tag >= 60:
            continue
        parent_name = joint.get_name_by_tag(parent_tag)
        children.setdefault(parent_name, []).append(child_name)
        parent_of[child_name] = parent_name

    def _compose(parent_tf, local_tf):
        ppos = np.array(parent_tf[0], dtype=np.float64)
        pq = np.array(
            [parent_tf[1][1], parent_tf[1][2], parent_tf[1][3], parent_tf[1][0]],
            dtype=np.float64,
        )
        pr = Rotation.from_quat(pq)
        lpos = np.array(local_tf[0], dtype=np.float64)
        lq = np.array(
            [local_tf[1][1], local_tf[1][2], local_tf[1][3], local_tf[1][0]],
            dtype=np.float64,
        )
        lr = Rotation.from_quat(lq)
        cpos = ppos + pr.apply(lpos)
        cr = pr * lr
        cq = cr.as_quat()
        return (
            (float(cpos[0]), float(cpos[1]), float(cpos[2])),
            (float(cq[3]), float(cq[0]), float(cq[1]), float(cq[2])),
        )

    right_hand_in_hips = None
    cur = "RightHand"
    chain: list[str] = []
    while cur in parent_of and cur != "Hips":
        chain.append(cur)
        cur = parent_of[cur]
    if cur == "Hips":
        tf = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        for name in reversed(chain):
            local_tf = local_transforms.get(name)
            if local_tf is None:
                tf = None
                break
            tf = _compose(tf, local_tf)
        right_hand_in_hips = tf

    rel: dict[str, tuple] = {
        "RightHand": right_hand_in_hips or ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        "_RightHandOrigin": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    }
    queue = deque(["RightHand"])

    while queue:
        parent_name = queue.popleft()
        parent_tf = rel.get(
            "_RightHandOrigin" if parent_name == "RightHand" else parent_name
        )
        if parent_tf is None:
            continue
        parent_pos_cm = np.array(parent_tf[0], dtype=np.float64)
        parent_q_xyzw = np.array(
            [parent_tf[1][1], parent_tf[1][2], parent_tf[1][3], parent_tf[1][0]],
            dtype=np.float64,
        )
        parent_rot = Rotation.from_quat(parent_q_xyzw)

        for child_name in children.get(parent_name, []):
            child_local = local_transforms.get(child_name)
            if child_local is None:
                continue
            child_pos_local = np.array(child_local[0], dtype=np.float64)
            child_q_local_xyzw = np.array(
                [
                    child_local[1][1],
                    child_local[1][2],
                    child_local[1][3],
                    child_local[1][0],
                ],
                dtype=np.float64,
            )
            child_rot_local = Rotation.from_quat(child_q_local_xyzw)

            child_pos_rel = parent_pos_cm + parent_rot.apply(child_pos_local)
            child_rot_rel = parent_rot * child_rot_local
            child_q_rel_xyzw = child_rot_rel.as_quat()
            rel[child_name] = (
                (
                    float(child_pos_rel[0]),
                    float(child_pos_rel[1]),
                    float(child_pos_rel[2]),
                ),
                (
                    float(child_q_rel_xyzw[3]),
                    float(child_q_rel_xyzw[0]),
                    float(child_q_rel_xyzw[1]),
                    float(child_q_rel_xyzw[2]),
                ),
            )
            queue.append(child_name)

    return rel


# ---------------------------------------------------------------------------
# Teleoperator
# ---------------------------------------------------------------------------


class MocapEefLeader(Teleoperator):
    """Mocap leader that emits absolute EE poses for the ``fr3_eef`` follower.

    No inverse kinematics is performed. Each cycle:

    1. Read the latest mocap RightHand pose (already in ROS-converted frame).
    2. Compute the delta vs the previous frame in mocap space.
    3. Optionally remap mocap axes to FR3 base axes (X<-Y, Y<-(-X), Z<-Z).
    4. Scale, low-pass filter, and clamp the delta.
    5. Accumulate the delta into the internal command pose, which is seeded
       on the first frame from the follower's ``arm_pose_state_topic``.
    6. Emit ``[ee_x, ee_y, ee_z, ori_r, ori_p, ori_y]`` (extrinsic xyz Euler)
       plus 6 hand joint positions in the same order as ``FR3EEFConfig``.
    """

    config_class = MocapEefLeaderConfig
    name = "mocap_eef_leader"

    def __init__(self, config: MocapEefLeaderConfig):
        super().__init__(config)
        self.config = config

        self._node = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._connected = False
        self._owns_rclpy = False
        self._lock = threading.Lock()

        # Mocap polling.
        self._mcp_app = None
        self._mocap_thread: threading.Thread | None = None
        self._mocap_stop = threading.Event()

        # Mocap RightHand pose: ((x, y, z), (qw, qx, qy, qz)) in ROS-converted frame.
        self._latest_arm_pose: Optional[
            tuple[tuple[float, float, float], tuple[float, float, float, float]]
        ] = None
        self._latest_finger_bends: dict[str, float] = {}
        self._prev_hand_pose: Optional[
            tuple[tuple[float, float, float], tuple[float, float, float, float]]
        ] = None

        # Latest measured EE pose from the follower (used to seed cmd state).
        self._arm_pose_msg: Optional[PoseStamped] = None

        # Internal absolute command pose (in FR3 base frame).
        self._cmd_pos: Optional[np.ndarray] = None  # (3,) meters
        self._cmd_quat_xyzw: Optional[np.ndarray] = None  # scipy ordering [x,y,z,w]

        # Per-cycle delta low-pass state.
        self._filtered_delta_x: Optional[np.ndarray] = None

        self._debug_counter = 0

    # ------------------------------------------------------------------
    # Feature schemas
    # ------------------------------------------------------------------
    @property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {f"{j}.pos": float for j in self.config.arm_pose_names}
        features.update({f"{j}.pos": float for j in self.config.hand_joint_names})
        return features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s does not require calibration", self)

    def configure(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected")

        if MCPApplication is None:
            raise ImportError("mocap_robotapi is required to use MocapEefLeader")

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        node_name = f"mocap_eef_leader_{self.id or 'default'}"
        self._node = rclpy.create_node(node_name)
        self._node.create_subscription(
            PoseStamped,
            self.config.arm_pose_state_topic,
            self._arm_pose_cb,
            10,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._mcp_app = MCPApplication()
        settings = MCPSettings()
        settings.set_udp(7012)
        settings.set_bvh_rotation(0)
        self._mcp_app.set_settings(settings)
        self._mcp_app.open()

        self._mocap_stop.clear()
        self._mocap_thread = threading.Thread(target=self._mocap_poll_loop, daemon=True)
        self._mocap_thread.start()

        self._connected = True
        logger.info(
            "%s 🚀 connected (arm_pose_state=%s)",
            self,
            self.config.arm_pose_state_topic,
        )

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self._mocap_stop.set()
        if self._mocap_thread is not None:
            self._mocap_thread.join(timeout=1.0)
            self._mocap_thread = None
        if self._mcp_app is not None:
            try:
                self._mcp_app.close()
            except Exception:
                pass
            self._mcp_app = None

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

    # ------------------------------------------------------------------
    # ROS / mocap callbacks
    # ------------------------------------------------------------------
    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._arm_pose_msg = msg

    def _mocap_poll_loop(self) -> None:
        assert self._mcp_app is not None
        period = 1.0 / float(self.config.mocap_poll_hz)
        while not self._mocap_stop.is_set():
            t0 = time.time()

            evts = self._mcp_app.poll_next_event()
            last_avatar = None
            for evt in evts:
                if evt.event_type == MCPEventType.AvatarUpdated:
                    last_avatar = MCPAvatar(evt.event_data.avatar_handle)

            if last_avatar is not None:
                local_transforms = build_local_transforms(last_avatar)
                cache: dict[str, tuple] = {}
                right_hand_tf = get_global_transform("RightHand", local_transforms, cache)
                if right_hand_tf is not None:
                    hand_pos, hand_quat = right_hand_tf
                    finger_bends = self._compute_hand_from_avatar(last_avatar)
                    with self._lock:
                        self._latest_arm_pose = (hand_pos, hand_quat)
                        self._latest_finger_bends = finger_bends

            elapsed = time.time() - t0
            remain = period - elapsed
            if remain > 0:
                time.sleep(remain)

    # ------------------------------------------------------------------
    # Mocap → action
    # ------------------------------------------------------------------
    def _compute_hand_from_avatar(self, avatar) -> dict[str, float]:
        """Same finger-bend extraction as ``MocapLeader._compute_hand_from_avatar``."""
        local_transforms_raw = _build_local_transforms(avatar)
        right_hand_rel = _build_right_hand_relative_transforms(
            avatar, local_transforms_raw
        )

        finger_bends: dict[str, float] = {}
        for tip_name, base_name, parent_name, child_name in FINGER_BASE_ANGLE_JOINTS:
            base_tf = right_hand_rel.get(base_name)
            parent_tf = right_hand_rel.get(parent_name)
            child_tf = right_hand_rel.get(child_name)
            if base_tf is None or parent_tf is None or child_tf is None:
                finger_bends[tip_name] = 0.0
                continue
            base_pos = np.array(base_tf[0], dtype=np.float64)
            parent_pos = np.array(parent_tf[0], dtype=np.float64)
            child_pos = np.array(child_tf[0], dtype=np.float64)
            v_parent = parent_pos - base_pos
            v_child = child_pos - base_pos
            n_p = float(np.linalg.norm(v_parent))
            n_c = float(np.linalg.norm(v_child))
            if n_p < 1e-6 or n_c < 1e-6:
                bend = 0.0
            else:
                cos_a = float(np.dot(v_parent, v_child) / (n_p * n_c))
                cos_a = max(-1.0, min(1.0, cos_a))
                angle = math.acos(cos_a)
                angle = max(ANGLE_CLOSED, min(ANGLE_OPEN, angle))
                bend = (
                    (ANGLE_OPEN - angle) / (ANGLE_OPEN - ANGLE_CLOSED)
                    if ANGLE_OPEN > ANGLE_CLOSED
                    else 0.0
                )
                bend = max(0.0, min(1.0, bend))
            finger_bends[tip_name] = bend

        thumb_tip_tf = right_hand_rel.get("RightHandThumb3")
        wrist_tf = right_hand_rel.get("_RightHandOrigin")
        mid_base_tf = right_hand_rel.get("RightInHandMiddle")
        idx_base_tf = right_hand_rel.get("RightInHandIndex")
        pky_base_tf = right_hand_rel.get("RightInHandPinky")
        thumb_rot_bend = 0.0
        if thumb_tip_tf and wrist_tf and mid_base_tf and idx_base_tf and pky_base_tf:
            thumb_fingertip = np.array(thumb_tip_tf[0], dtype=np.float64)
            wrist = np.array(wrist_tf[0], dtype=np.float64)
            joint4 = np.array(mid_base_tf[0], dtype=np.float64)
            joint5 = np.array(idx_base_tf[0], dtype=np.float64)
            joint6 = np.array(pky_base_tf[0], dtype=np.float64)
            palm_plane = [wrist, joint5, joint6]
            thumb_plane = [wrist, joint4, thumb_fingertip]
            angle = _calculate_plane_angle(palm_plane, thumb_plane)
            thumb_rot_bend = float(np.clip(angle / (math.pi / 2.0), 0.0, 1.0))
        finger_bends["_thumb_rotation"] = thumb_rot_bend
        return finger_bends

    def _wait_for_mocap(self) -> None:
        deadline = time.monotonic() + self.config.timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_arm_pose is not None:
                    return
            time.sleep(0.01)
        raise TimeoutError("Timeout waiting for mocap data from PNstudio")

    def _wait_for_arm_pose(self) -> None:
        deadline = time.monotonic() + self.config.timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._arm_pose_msg is not None:
                    return
            time.sleep(0.01)
        raise TimeoutError(
            f"Timeout waiting for FR3 EE pose on {self.config.arm_pose_state_topic}"
        )

    def reset_incremental_pose(self) -> None:
        """Reset incremental command state at episode boundaries.

        Re-seed ``_cmd_pos`` / ``_cmd_quat_xyzw`` from the follower's most
        recent measured pose so the very first frame of the new episode does
        not produce a large jump. Also clear filtered delta and previous
        mocap frame so mocap LPF does not carry across episodes.
        """
        with self._lock:
            self._cmd_pos = None
            self._cmd_quat_xyzw = None
            self._filtered_delta_x = None
            if self._latest_arm_pose is not None:
                hand_pos, hand_quat = self._latest_arm_pose
                self._prev_hand_pose = (
                    (hand_pos[0], hand_pos[1], hand_pos[2]),
                    (hand_quat[0], hand_quat[1], hand_quat[2], hand_quat[3]),
                )
            else:
                self._prev_hand_pose = None

    def _seed_cmd_pose_from_measured(self) -> None:
        """Initialize ``_cmd_pos`` / ``_cmd_quat_xyzw`` from the latest
        ``arm_pose_state_topic`` message. Caller must hold ``self._lock``."""
        msg = self._arm_pose_msg
        assert msg is not None, "expected `_wait_for_arm_pose` to have run already"
        self._cmd_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        self._cmd_quat_xyzw = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=np.float64,
        )

    def _compute_arm_pose_target(self) -> list[float]:
        """Return ``[x, y, z, roll, pitch, yaw]`` (extrinsic xyz Euler) target."""
        self._wait_for_mocap()
        self._wait_for_arm_pose()

        with self._lock:
            assert self._latest_arm_pose is not None
            hand_pos, hand_quat = self._latest_arm_pose

            # Seed the command pose from the follower's measured pose if
            # this is the first frame (or after a reset).
            if self._cmd_pos is None or self._cmd_quat_xyzw is None:
                self._seed_cmd_pose_from_measured()

            # Compute mocap delta (in mocap RightHand frame).
            if self._prev_hand_pose is None:
                delta_pos = (0.0, 0.0, 0.0)
                delta_quat_xyzw = (0.0, 0.0, 0.0, 1.0)
            else:
                prev_pos, prev_quat = self._prev_hand_pose
                delta_pos = (
                    hand_pos[0] - prev_pos[0],
                    hand_pos[1] - prev_pos[1],
                    hand_pos[2] - prev_pos[2],
                )
                # mocap_leader convention: hand_quat / prev_quat are stored as
                # (qx, qy, qz, qw) in ROS-converted mocap frame.
                # delta_q = q_curr * q_prev^{-1} (left-multiplication / world frame).
                delta_quat_xyzw = quat_multiply(hand_quat, quat_conjugate(prev_quat))

            self._prev_hand_pose = (hand_pos, hand_quat)

        delta_rotvec = Rotation.from_quat(
            np.array(delta_quat_xyzw, dtype=np.float64)
        ).as_rotvec()
        delta_pos_arr = np.array(delta_pos, dtype=np.float64)

        # Mocap → FR3 base axis mapping (same as MocapLeader).
        if self.config.enable_mocap_to_fr3_axis_mapping:
            aligned_delta_pos = np.array(
                [delta_pos_arr[1], -delta_pos_arr[0], delta_pos_arr[2]],
                dtype=np.float64,
            )
            aligned_delta_rotvec = np.array(
                [delta_rotvec[1], -delta_rotvec[0], delta_rotvec[2]],
                dtype=np.float64,
            )
            delta_x = np.hstack([aligned_delta_pos, aligned_delta_rotvec])
        else:
            delta_x = np.hstack([delta_pos_arr, delta_rotvec])

        # Per-cycle gains.
        delta_x[:3] *= float(self.config.delta_pos_gain)
        delta_x[3:] *= float(self.config.delta_rot_gain)

        # Low-pass on the delta stream to suppress mocap jitter.
        alpha = float(self.config.delta_lpf_alpha)
        if self._filtered_delta_x is None:
            self._filtered_delta_x = delta_x.copy()
        else:
            self._filtered_delta_x = (
                alpha * delta_x + (1.0 - alpha) * self._filtered_delta_x
            )
        delta_x = self._filtered_delta_x.copy()

        # Clamp filtered delta.
        max_dpos = float(self.config.max_delta_pos_per_cycle)
        max_drot = float(self.config.max_delta_rot_per_cycle)
        dpos_norm = float(np.linalg.norm(delta_x[:3]))
        if dpos_norm > max_dpos and dpos_norm > 1e-12:
            delta_x[:3] *= max_dpos / dpos_norm
        drot_norm = float(np.linalg.norm(delta_x[3:]))
        if drot_norm > max_drot and drot_norm > 1e-12:
            delta_x[3:] *= max_drot / drot_norm

        # Accumulate into absolute command pose, in FR3 base frame.
        # Translation: cmd_pos_new = cmd_pos + delta_pos_in_base
        # Rotation:    R_new = exp(delta_rotvec_in_base) @ R_current
        # which matches MocapLeader's IK target convention exactly:
        #   target_r = pin.exp3(delta_x[3:]) @ o_m_f0.rotation
        with self._lock:
            assert self._cmd_pos is not None and self._cmd_quat_xyzw is not None
            self._cmd_pos = self._cmd_pos + delta_x[:3]
            current_rot = Rotation.from_quat(self._cmd_quat_xyzw)
            new_rot = Rotation.from_rotvec(delta_x[3:]) * current_rot
            self._cmd_quat_xyzw = new_rot.as_quat()
            cmd_pos = self._cmd_pos.copy()

        # Convert to extrinsic xyz Euler — matches `fr3_eef.send_action`'s
        # `Rotation.from_euler("xyz", target_euler)` round-trip.
        euler = new_rot.as_euler("xyz")

        self._debug_counter += 1
        return [
            float(cmd_pos[0]),
            float(cmd_pos[1]),
            float(cmd_pos[2]),
            float(euler[0]),
            float(euler[1]),
            float(euler[2]),
        ]

    def _compute_hand_joints(self) -> list[float]:
        """Same hand bend → LinkerHand L6 mapping as MocapLeader."""
        with self._lock:
            finger_bends = dict(self._latest_finger_bends)

        # bend v01: 0=open, 1=closed → LinkerHand L6 position: 0=closed, 255=open
        def _to_u8_open(v01: float) -> float:
            v01 = float(np.clip(v01, 0.0, 1.0))
            return float(255.0 * (1.0 - v01))

        thumb_flex = float(finger_bends.get("RightHandThumb3", 0.0))
        thumb_abd = float(finger_bends.get("_thumb_rotation", 0.0))
        index_flex = float(finger_bends.get("RightHandIndex3", 0.0))
        middle_flex = float(finger_bends.get("RightHandMiddle3", 0.0))
        ring_flex = float(finger_bends.get("RightHandRing3", 0.0))
        little_flex = float(finger_bends.get("RightHandPinky3", 0.0))

        joints = [
            _to_u8_open(thumb_flex),
            _to_u8_open(thumb_abd),
            _to_u8_open(index_flex),
            _to_u8_open(middle_flex),
            _to_u8_open(ring_flex),
            _to_u8_open(little_flex),
        ]
        return joints[: len(self.config.hand_joint_names)]

    # ------------------------------------------------------------------
    # Teleoperator API
    # ------------------------------------------------------------------
    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        arm_pose = self._compute_arm_pose_target()
        hand_pos = self._compute_hand_joints()

        action: dict[str, float] = {}
        for idx, name in enumerate(self.config.arm_pose_names):
            action[f"{name}.pos"] = arm_pose[idx]
        for idx, name in enumerate(self.config.hand_joint_names):
            action[f"{name}.pos"] = hand_pos[idx] if idx < len(hand_pos) else 0.0
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
