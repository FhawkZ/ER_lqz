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

"""MocapRetargetLeader: EE 增量控制沿用 ``MocapEefLeader``，手部用 dex-retargeting 解算 Linker L6。"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Optional

import numpy as np

from lerobot.teleoperators.mocap_eef_leader.mocap_eef_leader import (
    MCPAvatar,
    MCPEventType,
    MocapEefLeader,
    build_local_transforms,
    get_global_transform,
)

from .config_mocap_retarget_leader import MocapRetargetLeaderConfig
from .src.dex_retargeter import DexRetargeter, default_dex_paths
from .src.linker_joint_cmd import dex_qpos_to_hand_cmd255
from .src.mocap_reader import MocapReader, MocapSkeletonFrame
from .src.utils import frame_to_dex_joint_pos

logger = logging.getLogger(__name__)


class MocapRetargetLeader(MocapEefLeader):
    """动捕 leader：臂部与 ``MocapEefLeader`` 相同，手部由 dex-retargeting 映射到 Linker L6。

    输出 schema 与 ``mocap_eef_leader`` 一致（6D EE + ``hand_0``…``hand_5``），配对
    ``fr3_eef`` follower。
    """

    config_class = MocapRetargetLeaderConfig
    name = "mocap_retarget_leader"

    def __init__(self, config: MocapRetargetLeaderConfig):
        super().__init__(config)
        self.config: MocapRetargetLeaderConfig = config

        dex_repo, urdf_root, dex_cfg = default_dex_paths()
        if config.dex_repo:
            dex_repo = Path(config.dex_repo).resolve()
        if config.urdf_root:
            urdf_root = Path(config.urdf_root).resolve()
        if config.dex_config:
            dex_cfg = Path(config.dex_config).resolve()

        self._dex = DexRetargeter(dex_repo, urdf_root, dex_cfg)
        self._mocap_reader = MocapReader(poll_hz=float(config.mocap_poll_hz))
        self._latest_skeleton_frame: Optional[MocapSkeletonFrame] = None

        logger.info(
            "%s dex: repo=%s urdf_root=%s urdf=%s config=%s",
            self,
            dex_repo or "(pip)",
            urdf_root,
            self._dex.resolved_urdf_path,
            dex_cfg,
        )

    def _mocap_poll_loop(self) -> None:
        """与父类相同地更新腕部位姿，并缓存骨架帧供 dex 手部解算。"""
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
                    frame = self._mocap_reader.parse_avatar(last_avatar)
                    with self._lock:
                        self._latest_arm_pose = (hand_pos, hand_quat)
                        self._latest_skeleton_frame = frame

            elapsed = time.time() - t0
            remain = period - elapsed
            if remain > 0:
                time.sleep(remain)

    def _compute_hand_joints(self) -> list[float]:
        """dex-retargeting：MCP 21 点 -> 6 关节弧度 -> ``hand_0``…``hand_5`` 的 0–255。"""
        with self._lock:
            frame = self._latest_skeleton_frame

        n = len(self.config.hand_joint_names)
        if frame is None:
            return [0.0] * n

        joint_pos = frame_to_dex_joint_pos(
            self._mocap_reader,
            frame,
            hand_side=self.config.hand_side,
            position_scale=float(self.config.dex_position_scale),
        )
        _, robot_qpos = self._dex.retarget(joint_pos)
        q = self._dex.target_qpos(robot_qpos)
        cmd = dex_qpos_to_hand_cmd255(q, self._dex.target_joint_names)
        return cmd[:n]
