"""Linker L6 主动关节弧度 -> 0–255。"""

from __future__ import annotations

import numpy as np

# URDF 限位（弧度）；invert=True：255 对应伸直侧
_JOINT_LIMITS: dict[str, tuple[float, float, bool]] = {
    "rh_thumb_cmc_roll": (-0.087266, 1.256637, True),
    "rh_thumb_cmc_pitch": (0.0, 0.837758, True),
    "rh_index_mcp_pitch": (0.0, 1.134464, True),
    "rh_middle_mcp_pitch": (0.0, 1.134464, True),
    "rh_ring_mcp_pitch": (0.0, 1.134464, True),
    "rh_pinky_mcp_pitch": (0.0, 1.134464, True),
}


# dex target_joint_names -> FR3LinkerL6Follower hand_0..hand_5 顺序
# hand_0: 拇指屈伸, hand_1: 拇指外展, hand_2..5: 食/中/无名/小指
_DEX_TO_HAND_INDEX: dict[str, int] = {
    "rh_thumb_cmc_pitch": 0,
    "rh_thumb_cmc_roll": 1,
    "rh_index_mcp_pitch": 2,
    "rh_middle_mcp_pitch": 3,
    "rh_ring_mcp_pitch": 4,
    "rh_pinky_mcp_pitch": 5,
}


def dex_qpos_to_hand_cmd255(qpos_rad: np.ndarray, dex_joint_names: list[str]) -> list[float]:
    """将 dex 关节弧度按名称映射到 ``hand_0``…``hand_5`` 的 0–255 指令。"""
    cmd = rad_to_cmd255(qpos_rad, dex_joint_names)
    out = [0.0] * 6
    for i, name in enumerate(dex_joint_names):
        idx = _DEX_TO_HAND_INDEX.get(name)
        if idx is None:
            raise KeyError(f"未知 dex 关节名，无法映射到 hand_*：{name}")
        out[idx] = float(cmd[i])
    return out


def rad_to_cmd255(qpos_rad: np.ndarray, joint_names: list[str]) -> np.ndarray:
    q = np.asarray(qpos_rad, dtype=np.float64).reshape(-1)
    if q.shape[0] != len(joint_names):
        raise ValueError(f"qpos 长度 {q.shape[0]} 与关节数 {len(joint_names)} 不一致。")

    cmd = np.empty(len(joint_names), dtype=np.int32)
    for i, name in enumerate(joint_names):
        lo, hi, invert = _JOINT_LIMITS[name]
        v = float(np.clip(q[i], lo, hi))
        span = hi - lo
        ratio = (hi - v) / span if invert else (v - lo) / span
        cmd[i] = int(np.round(np.clip(ratio, 0.0, 1.0) * 255.0))
    return cmd
