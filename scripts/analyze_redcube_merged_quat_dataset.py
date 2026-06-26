#!/usr/bin/env python3
"""分析训练数据集 redcube_merged_quat（动作平滑度、四元数、手指范围等）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

ARM = slice(0, 7)
QUAT = slice(3, 7)
HAND = slice(7, 13)

ACTION_NAMES = [
    "ee_x", "ee_y", "ee_z", "ori_qx", "ori_qy", "ori_qz", "ori_qw",
    "hand_0", "hand_1", "hand_2", "hand_3", "hand_4", "hand_5",
]


def _load_parquet_frames(dataset_root: Path) -> pd.DataFrame:
    files = sorted((dataset_root / "data").glob("*/*.parquet"))
    if not files:
        raise FileNotFoundError(f"no parquet under {dataset_root}/data")
    return pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)


def _step_delta(arr: np.ndarray) -> np.ndarray:
    return np.abs(np.diff(arr, axis=0))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data/redcube_merged_quat",
    )
    args = parser.parse_args()
    root = args.dataset_root.resolve()
    if not (root / "meta/info.json").is_file():
        print(f"FAIL missing dataset: {root}", file=sys.stderr)
        return 1

    with open(root / "meta/info.json") as f:
        info = json.load(f)
    df = _load_parquet_frames(root)

    print(f"=== 数据集: {root} ===")
    print(f"episodes={info['total_episodes']} frames={info['total_frames']} fps={info['fps']}")
    print(f"robot_type={info.get('robot_type', '?')}")

    actions = np.stack(df["action"].values)
    states = np.stack(df["observation.state"].values)
    assert actions.shape[1] == 13 and states.shape[1] == 13

    # 四元数单位范数
    for label, arr in ("action", actions), ("state", states):
        norms = np.linalg.norm(arr[:, QUAT], axis=1)
        print(f"\n[{label} quat] norm mean={norms.mean():.6f} max_err={np.max(np.abs(norms - 1)):.6f}")

    # 动作逐步变化（平滑度 proxy）
    d_action = _step_delta(actions)
    print("\n[动作逐步 |Δ| 均值] (越小越平滑)")
    for i, name in enumerate(ACTION_NAMES):
        print(f"  {name:8s}: mean={d_action[:, i].mean():.5f}  p95={np.percentile(d_action[:, i], 95):.5f}")

    # 手臂 vs 手指
    pos_d = d_action[:, :3].mean(axis=1)
    hand_d = d_action[:, HAND].mean(axis=1)
    print(f"\n[pos xyz 逐步均值] mean={pos_d.mean()*1000:.2f}mm  p95={np.percentile(pos_d,95)*1000:.2f}mm")
    print(f"[hand 逐步均值]   mean={hand_d.mean():.2f}  p95={np.percentile(hand_d,95):.2f}")

    # action vs state 同帧偏差
    geo = np.degrees(
        (Rotation.from_quat(actions[:, QUAT]) * Rotation.from_quat(states[:, QUAT]).inv()).magnitude()
    )
    pos_err = np.linalg.norm(actions[:, :3] - states[:, :3], axis=1)
    print(f"\n[同帧 action vs state]")
    print(f"  pos L2 (m): mean={pos_err.mean():.4f} p95={np.percentile(pos_err,95):.4f}")
    print(f"  ori geodesic (deg): mean={geo.mean():.2f} p95={np.percentile(geo,95):.2f}")

    # episode 内统计
    ep_hand_std = []
    for ep in df["episode_index"].unique()[:20]:
        m = df["episode_index"] == ep
        ep_a = np.stack(df.loc[m, "action"].values)
        ep_hand_std.append(ep_a[:, HAND].std(axis=0).mean())
    print(f"\n[前20 ep 手指关节 std 均值] mean={np.mean(ep_hand_std):.1f}")

    with open(root / "meta/stats.json") as f:
        stats = json.load(f)
    if "action" in stats:
        hand_std = stats["action"]["std"][7:13]
        print(f"[meta/stats action hand std] {hand_std}")

    print("\n=== 完成 ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
