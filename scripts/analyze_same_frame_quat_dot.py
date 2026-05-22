#!/usr/bin/env python3
"""Same-frame action vs obs: geodesic angle vs quaternion dot (sign hemisphere)."""

from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

SRC = Path("/media/data/liqz/data/redcube_merged")
DST = Path("/media/data/liqz/data/redcube_merged_quat")


def euler_to_quat(v12: np.ndarray) -> np.ndarray:
    return Rotation.from_euler("xyz", v12[3:6]).as_quat()


def analyze(df: pd.DataFrame, label: str, act_quats: np.ndarray, obs_quats: np.ndarray) -> None:
    geo = np.degrees(
        (Rotation.from_quat(act_quats).inv() * Rotation.from_quat(obs_quats)).magnitude()
    )
    dots = np.sum(act_quats * obs_quats, axis=1)

    n = len(dots)
    neg = dots < 0
    print(f"\n=== {label} (n={n}) ===")
    print(f"geodesic: mean={geo.mean():.2f}° p95={np.percentile(geo, 95):.2f}° max={geo.max():.2f}°")
    print(f"dot<0: {neg.sum()} ({100 * neg.mean():.2f}%)")

    for thresh in (15, 30, 90):
        mask = geo < thresh
        if not mask.any():
            continue
        sub_neg = (dots[mask] < 0).sum()
        print(
            f"  geo<{thresh}°: {mask.sum()} frames, dot<0: {sub_neg} "
            f"({100 * sub_neg / mask.sum():.2f}%)"
        )

    if neg.any():
        idx = np.where(neg)[0][:5]
        print("  examples (geo_deg, dot, ep, frame):")
        for i in idx:
            ep = int(df.iloc[i]["episode_index"])
            fi = int(df.iloc[i]["frame_index"]) if "frame_index" in df.columns else -1
            print(f"    geo={geo[i]:.2f}° dot={dots[i]:.4f} ep={ep} frame={fi}")


def main() -> None:
    df = pd.read_parquet(next((SRC / "data").glob("*/*.parquet")))

    act_e = np.stack(df["action"].values)
    obs_e = np.stack(df["observation.state"].values)

    # 1) 原 Euler：各自转 quat，无符号对齐
    act_q = np.stack([euler_to_quat(a) for a in act_e])
    obs_q = np.stack([euler_to_quat(s) for s in obs_e])
    analyze(df, "原数据 naive euler→quat（各自独立）", act_q, obs_q)

    # 2) 最优符号配对：在 q 与 -q 中选使 dot 最大的组合（几何上合理上界）
    best_dots = []
    for i in range(len(act_q)):
        d0 = float(np.dot(act_q[i], obs_q[i]))
        d1 = float(np.dot(-act_q[i], obs_q[i]))
        best_dots.append(max(d0, d1))
    best_dots = np.array(best_dots)
    geo = np.degrees(
        (Rotation.from_quat(act_q).inv() * Rotation.from_quat(obs_q)).magnitude()
    )
    print("\n=== 若每帧允许 action 取 q 或 -q（旋转不变，最优配对）===")
    print(f"dot<0 剩余: {(best_dots < 0).sum()} (应≈0)")
    print(f"geo<15° 且最优 dot<0: {((geo < 15) & (best_dots < 0)).sum()}")

    # 3) 迁移后数据集（含 global ref + sync）
    ddf = pd.read_parquet(next((DST / "data").glob("*/*.parquet")))
    act_m = np.stack(ddf["action"].values)[:, 3:7]
    obs_m = np.stack(ddf["observation.state"].values)[:, 3:7]
    analyze(ddf, "迁移后 redcube_merged_quat", act_m, obs_m)


if __name__ == "__main__":
    main()
