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

from dataclasses import dataclass, field

from lerobot.teleoperators.mocap_eef_leader.config_mocap_eef_leader import MocapEefLeaderConfig

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("mocap_retarget_leader")
@dataclass
class MocapRetargetLeaderConfig(MocapEefLeaderConfig):
    """Mocap leader：臂部沿用 ``MocapEefLeader`` 的 EE 增量控制，手部用 dex-retargeting 解算 Linker L6。

    与 ``mocap_eef_leader`` 输出 schema 相同（6D EE + 6D hand），配对 ``fr3_eef``
    （笛卡尔阻抗臂 + Linker L6 手）。臂部关节轨迹请用 ``mocap_leader`` +
    ``fr3_linker_l6_follower``。
    """

    # dex-retargeting：空字符串表示 pip 安装或 ``teleoperators/dex-retargeting`` 源码
    dex_repo: str = ""
    # URDF 根目录；空字符串表示 ``mocap_retarget/assets``（内含 l6/right/...urdf）
    urdf_root: str = ""
    dex_config: str = ""

    # MCP 右手局部系 -> dex 21 点时的尺度（与 ``run_realtime_retarget`` 一致，单位 m）
    dex_position_scale: float = 0.01
    hand_side: str = "right"
