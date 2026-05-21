#!/usr/bin/env python3
"""分析 infer 轨迹 CSV，重点看姿态 (ori_r/p/y) 与策略-当前位姿偏差。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


ARM_COLS = ["ee_x", "ee_y", "ee_z", "ori_r", "ori_p", "ori_y"]


def _euler_cols(prefix: str) -> list[str]:
    return [f"{prefix}_{c}" for c in ARM_COLS]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace_dir", type=Path, help="INFER_TRACE_DIR（含 episode_*.csv）")
    args = parser.parse_args()
    trace_dir = args.trace_dir
    csv_files = sorted(trace_dir.glob("episode_*.csv"))
    if not csv_files:
        print(f"未找到 CSV: {trace_dir}/episode_*.csv", file=sys.stderr)
        return 1

    for csv_path in csv_files:
        df = pd.read_csv(csv_path)
        print(f"\n=== {csv_path.name} ({len(df)} 帧) ===")
        if "loop_hz" in df.columns:
            print(f"  实际控制频率: median={df['loop_hz'].median():.1f} Hz, min={df['loop_hz'].min():.1f} Hz")

        if "ori_delta_deg" in df.columns:
            print(
                f"  策略姿态 vs 当前姿态 (测地角): mean={df['ori_delta_deg'].mean():.1f}°, "
                f"max={df['ori_delta_deg'].max():.1f}°, p95={df['ori_delta_deg'].quantile(0.95):.1f}°"
            )

        for prefix, label in (("state", "当前"), ("policy", "策略"), ("sent", "下发")):
            cols = _euler_cols(prefix)
            if not all(c in df.columns for c in cols):
                continue
            sub = df[cols[3:6]]
            print(f"  {label} ori_rpy (rad): r=[{sub.iloc[:, 0].min():.2f},{sub.iloc[:, 0].max():.2f}] "
                  f"p=[{sub.iloc[:, 1].min():.2f},{sub.iloc[:, 1].max():.2f}] "
                  f"y=[{sub.iloc[:, 2].min():.2f},{sub.iloc[:, 2].max():.2f}]")

        # 检测 ori_r 是否在 ±π 两侧跳变（欧拉多值性）
        if "policy_ori_r" in df.columns:
            jumps = np.abs(np.diff(df["policy_ori_r"].to_numpy()))
            big = jumps > 2.0
            if big.any():
                idx = np.where(big)[0]
                print(f"  策略 ori_r 大跳变 {big.sum()} 次 (|Δ|>2 rad)，例如帧 {idx[:5].tolist()} — 可能是欧拉角分支切换")

        # 策略相对当前的 roll 差（未考虑四元数等价）
        if "policy_ori_r" in df.columns and "state_ori_r" in df.columns:
            dr = df["policy_ori_r"] - df["state_ori_r"]
            print(f"  policy_ori_r - state_ori_r: mean={dr.mean():.2f} rad, std={dr.std():.2f} rad")

    print(
        "\n训练集 action 典型 ori_r: mean≈0.96, q50≈1.42, q10≈-1.77 (redcube_merged)。"
        "\n若 policy_ori_r 常落在 ±π 附近且 ori_delta_deg 很大，说明策略在「翻腕」分支上输出，与当前欧拉表示不一致。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
