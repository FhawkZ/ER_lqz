#!/usr/bin/env python3
"""分析 infer 轨迹 CSV，重点看姿态四元数与策略-当前位姿偏差。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


ARM_COLS = ["ee_x", "ee_y", "ee_z", "ori_qx", "ori_qy", "ori_qz", "ori_qw"]


def _arm_cols(prefix: str) -> list[str]:
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
            cols = _arm_cols(prefix)
            if not all(c in df.columns for c in cols):
                continue
            sub = df[cols[3:7]]
            print(
                f"  {label} quat: qw=[{sub.iloc[:, 3].min():.3f},{sub.iloc[:, 3].max():.3f}] "
                f"qx=[{sub.iloc[:, 0].min():.3f},{sub.iloc[:, 0].max():.3f}]"
            )

        # 检测四元数符号翻转（连续表示下应很少出现）
        if "policy_ori_qw" in df.columns:
            qw = df["policy_ori_qw"].to_numpy()
            flips = np.sum(np.abs(np.diff(qw)) > 1.5)
            if flips:
                print(f"  策略 ori_qw 疑似符号翻转 {flips} 次 (|Δqw|>1.5)")

    print(
        "\n数据集 EE 姿态现为 ori_qx..ori_qw（单位四元数，episode 内符号对齐）。"
        "\n若 ori_delta_deg 很大，多为策略误差而非 ±π 欧拉分支跳变。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
