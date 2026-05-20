"""MCP 动捕：UDP 读帧、FK、手部子树相对位姿。"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from ..mocap import MCPApplication, MCPAvatar, MCPEventType, MCPSettings

# parent tag 失败时回退
LINKS_PARENT: dict[str, str] = {
    "Hips": "world",
    "RightUpLeg": "Hips",
    "RightLeg": "RightUpLeg",
    "RightFoot": "RightLeg",
    "RightTiptoe": "RightFoot",
    "LeftUpLeg": "Hips",
    "LeftLeg": "LeftUpLeg",
    "LeftFoot": "LeftLeg",
    "LeftTiptoe": "LeftFoot",
    "Spine": "Hips",
    "Spine1": "Spine",
    "Spine2": "Spine1",
    "Neck": "Spine2",
    "Neck1": "Neck",
    "Head": "Neck1",
    "Head1": "Head",
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
    "LeftShoulder": "Spine2",
    "LeftArm": "LeftShoulder",
    "LeftForeArm": "LeftArm",
    "LeftHand": "LeftForeArm",
    "LeftHandThumb1": "LeftHand",
    "LeftHandThumb2": "LeftHandThumb1",
    "LeftHandThumb3": "LeftHandThumb2",
    "LeftInHandIndex": "LeftHand",
    "LeftHandIndex1": "LeftInHandIndex",
    "LeftHandIndex2": "LeftHandIndex1",
    "LeftHandIndex3": "LeftHandIndex2",
    "LeftInHandMiddle": "LeftHand",
    "LeftHandMiddle1": "LeftInHandMiddle",
    "LeftHandMiddle2": "LeftHandMiddle1",
    "LeftHandMiddle3": "LeftHandMiddle2",
    "LeftInHandRing": "LeftHand",
    "LeftHandRing1": "LeftInHandRing",
    "LeftHandRing2": "LeftHandRing1",
    "LeftHandRing3": "LeftHandRing2",
    "LeftInHandPinky": "LeftHand",
    "LeftHandPinky1": "LeftInHandPinky",
    "LeftHandPinky2": "LeftHandPinky1",
    "LeftHandPinky3": "LeftHandPinky2",
}


@dataclass(frozen=True)
class LocalTransform:
    position: tuple[float, float, float]
    rotation_wxyz: tuple[float, float, float, float]

    @property
    def rotation(self) -> Rotation:
        w, x, y, z = self.rotation_wxyz
        return Rotation.from_quat([x, y, z, w])


def _compose(parent: LocalTransform, child: LocalTransform) -> LocalTransform:
    pr, cr = parent.rotation, child.rotation
    pp = np.asarray(parent.position, dtype=np.float64)
    cp = np.asarray(child.position, dtype=np.float64)
    gp = pp + pr.apply(cp)
    q = (pr * cr).as_quat(scalar_first=True)
    return LocalTransform(
        (float(gp[0]), float(gp[1]), float(gp[2])),
        (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
    )


@dataclass
class MocapSkeletonFrame:
    timestamp_s: float
    root_joint_name: str
    local_transforms: dict[str, LocalTransform]
    global_transforms: dict[str, LocalTransform]
    joint_children: dict[str, list[str]] = field(default_factory=dict)
    joint_parent: dict[str, str] = field(default_factory=dict)


class MocapReader:
    def __init__(
        self,
        udp_port: int = 7012,
        bvh_rotation: int = 0,
        poll_hz: float = 120.0,
        position_scale: float = 1.0,
    ) -> None:
        self.udp_port = udp_port
        self.bvh_rotation = bvh_rotation
        self.poll_hz = poll_hz
        self.position_scale = position_scale
        self._mcp_app: Optional[MCPApplication] = None
        self._joint_children: Optional[dict[str, list[str]]] = None
        self._joint_parent: Optional[dict[str, str]] = None

    def open(self) -> None:
        if self._mcp_app is not None:
            return
        app = MCPApplication()
        settings = MCPSettings()
        settings.set_udp(self.udp_port)
        settings.set_bvh_rotation(self.bvh_rotation)
        app.set_settings(settings)
        app.open()
        self._mcp_app = app

    def close(self) -> None:
        if self._mcp_app is None:
            return
        try:
            self._mcp_app.close()
        finally:
            self._mcp_app = None
            self._joint_children = None
            self._joint_parent = None

    def _local_transforms(self, avatar: MCPAvatar) -> dict[str, LocalTransform]:
        scale = self.position_scale
        out: dict[str, LocalTransform] = {}
        for joint in avatar.get_joints():
            name = joint.get_name()
            lp = joint.get_local_position() or (0.0, 0.0, 0.0)
            lr = joint.get_local_rotation()
            out[name] = LocalTransform(
                (lp[0] * scale, lp[1] * scale, lp[2] * scale),
                lr,
            )
        return out

    def _hierarchy(self, avatar: MCPAvatar) -> tuple[dict[str, list[str]], dict[str, str]]:
        children: dict[str, list[str]] = {}
        parent_of: dict[str, str] = {}
        for joint in avatar.get_joints():
            child = joint.get_name()
            tag = joint.get_tag()
            try:
                ptag = joint.get_parent_joint_tag(tag)
            except RuntimeError:
                ptag = -1
            if ptag < 0 or ptag >= 60:
                fb = LINKS_PARENT.get(child)
                if fb and fb != "world":
                    parent_of[child] = fb
                    children.setdefault(fb, []).append(child)
                continue
            parent = joint.get_name_by_tag(ptag)
            parent_of[child] = parent
            children.setdefault(parent, []).append(child)
        return children, parent_of

    def _fk(
        self,
        local: dict[str, LocalTransform],
        root: str,
        children: dict[str, list[str]],
    ) -> dict[str, LocalTransform]:
        global_tf = {root: local[root]}
        queue = deque([root])
        while queue:
            parent = queue.popleft()
            pt = global_tf[parent]
            for child in children.get(parent, []):
                cl = local.get(child)
                if cl is None:
                    continue
                global_tf[child] = _compose(pt, cl)
                queue.append(child)
        return global_tf

    def parse_avatar(self, avatar: MCPAvatar) -> MocapSkeletonFrame:
        if self._joint_children is None:
            self._joint_children, self._joint_parent = self._hierarchy(avatar)

        local = self._local_transforms(avatar)
        if "Hips" in local and "Hips" not in self._joint_parent:
            root = "Hips"
        else:
            root = next(n for n in local if n not in self._joint_parent)

        return MocapSkeletonFrame(
            timestamp_s=time.time(),
            root_joint_name=root,
            local_transforms=local,
            global_transforms=self._fk(local, root, self._joint_children),
            joint_children=dict(self._joint_children),
            joint_parent=dict(self._joint_parent),
        )

    def read_frame(self, timeout_s: float = 2.0) -> MocapSkeletonFrame:
        if self._mcp_app is None:
            raise RuntimeError("请先调用 open()。")
        deadline = time.time() + timeout_s
        sleep_s = 1.0 / max(self.poll_hz, 1.0)
        while time.time() < deadline:
            last_avatar = None
            for event in self._mcp_app.poll_next_event():
                if event.event_type == MCPEventType.AvatarUpdated:
                    last_avatar = MCPAvatar(event.event_data.avatar_handle)
            if last_avatar is not None:
                return self.parse_avatar(last_avatar)
            time.sleep(sleep_s)
        raise TimeoutError(f"{timeout_s:.2f}s 内未收到 AvatarUpdated。")

    def build_subtree_relative_transforms(
        self,
        frame: MocapSkeletonFrame,
        root_name: str,
    ) -> dict[str, LocalTransform]:
        if root_name not in frame.global_transforms:
            raise KeyError(f"缺少关节 {root_name}")

        rel: dict[str, LocalTransform] = {
            root_name: LocalTransform((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        }
        queue = deque([root_name])
        while queue:
            parent = queue.popleft()
            pt = rel[parent]
            for child in frame.joint_children.get(parent, []):
                cl = frame.local_transforms.get(child)
                if cl is None:
                    continue
                rel[child] = _compose(pt, cl)
                queue.append(child)
        return rel
