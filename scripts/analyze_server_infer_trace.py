#!/usr/bin/env python3
"""分析 policy_server 推理 trace（logs/inference_traces 下的 obs_*/actions.csv）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


ARM = ["ee_x", "ee_y", "ee_z", "ori_qx", "ori_qy", "ori_qz", "ori_qw"]
HAND = [f"hand_{i}" for i in range(6)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "trace_root",
        type=Path,
        nargs="?",
        default=Path(__file__).resolve().parents[1] / "logs/inference_traces",
    )
    args = parser.parse_args()
    root = args.trace_root
    obs_dirs = sorted(
        p for p in root.glob("**/obs_*") if p.is_dir() and "errors" not in p.parts
    )
    if not obs_dirs:
        print(f"未找到 trace: {root}/**/obs_*", file=sys.stderr)
        return 1

    sessions = sorted({p.parent for p in obs_dirs})
    print(f"=== Server inference traces: {root} ({len(sessions)} session(s), {len(obs_dirs)} inferences) ===")
    for session in sessions:
        if (session / "session.json").is_file():
            session_meta = json.loads((session / "session.json").read_text())
            print(f"  session: {session_meta.get('session_id')} started {session_meta.get('started_at_utc')}")
    for obs_dir in obs_dirs:
        meta_path = obs_dir / "meta.json"
        csv_path = obs_dir / "actions.csv"
        if not csv_path.is_file():
            continue
        meta = json.loads(meta_path.read_text()) if meta_path.is_file() else {}
        df = pd.read_csv(csv_path)
        print(f"\n--- {obs_dir.relative_to(root)} policy={meta.get('policy_type')} t={meta.get('timestep')} ---")
        if "ee_x" in df.columns:
            pos_step = df[["ee_x", "ee_y", "ee_z"]].diff().abs().mean(axis=1).dropna()
            print(f"  chunk pos |Δ|/step (mm): mean={pos_step.mean()*1000:.2f} max={pos_step.max()*1000:.2f}")
        if all(c in df.columns for c in HAND):
            hand_step = df[HAND].diff().abs().mean(axis=1).dropna()
            print(f"  chunk hand |Δ|/step: mean={hand_step.mean():.1f} max={hand_step.max():.1f}")
        n_img = len(list((obs_dir / "images").glob("*.png"))) if (obs_dir / "images").is_dir() else 0
        print(f"  images saved: {n_img}  action steps in chunk: {len(df)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
