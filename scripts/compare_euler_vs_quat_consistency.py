#!/usr/bin/env python3
"""Compare Euler inconsistency in source vs quaternion migrated dataset."""

from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

SRC = Path("/media/data/liqz/data/redcube_merged")
DST = Path("/media/data/liqz/data/redcube_merged_quat")


def euler_jump_stats(df, key: str) -> np.ndarray:
    ori_sl = slice(3, 6)
    jumps = []
    for ep in sorted(df["episode_index"].unique()):
        m = df["episode_index"] == ep
        arr = np.stack(df.loc[m, key].values)
        for i in range(1, len(arr)):
            r0 = Rotation.from_euler("xyz", arr[i - 1, ori_sl])
            r1 = Rotation.from_euler("xyz", arr[i, ori_sl])
            jumps.append(np.degrees((r0.inv() * r1).magnitude()))
    return np.array(jumps)


def quat_dot_breaks(df, key: str) -> tuple[int, int]:
    breaks = total = 0
    for ep in sorted(df["episode_index"].unique()):
        m = df["episode_index"] == ep
        arr = np.stack(df.loc[m, key].values)
        for i in range(1, len(arr)):
            total += 1
            if np.dot(arr[i, 3:7], arr[i - 1, 3:7]) < 0:
                breaks += 1
    return breaks, total


def naive_quat_breaks_from_euler(df, key: str) -> tuple[int, int]:
    breaks = total = 0
    for ep in sorted(df["episode_index"].unique()):
        m = df["episode_index"] == ep
        arr = np.stack(df.loc[m, key].values)
        prev = None
        for i in range(len(arr)):
            q = Rotation.from_euler("xyz", arr[i, 3:6]).as_quat()
            if prev is not None:
                total += 1
                if np.dot(q, prev) < 0:
                    breaks += 1
            prev = q
    return breaks, total


def main() -> None:
    src_df = pd.read_parquet(next((SRC / "data").glob("*/*.parquet")))
    dst_df = pd.read_parquet(next((DST / "data").glob("*/*.parquet")))

    print("=== 原数据集 (Euler) 帧间问题 ===")
    for key in ("observation.state", "action"):
        j = euler_jump_stats(src_df, key)
        big90 = int((j > 90).sum())
        big170 = int((j > 170).sum())
        print(
            f"{key}: median={np.median(j):.2f}° p99={np.percentile(j, 99):.2f}° max={j.max():.2f}°"
        )
        print(f"  真实大转角(>90°): {big90}/{len(j)} ({100 * big90 / len(j):.2f}%)")
        print(f"  疑似分支跳变(>170°): {big170}/{len(j)} ({100 * big170 / len(j):.2f}%)")
        b, t = naive_quat_breaks_from_euler(src_df, key)
        print(f"  若直接 euler→quat(不对齐): 符号翻转 {b}/{t} ({100 * b / t:.2f}%)")

    print("\n=== 迁移后 (quat + align) 帧间 ===")
    for key in ("observation.state", "action"):
        b, t = quat_dot_breaks(dst_df, key)
        print(f"{key}: 符号翻转 q·q_prev<0: {b}/{t}")

    mis = 0
    for ep in sorted(dst_df["episode_index"].unique()):
        m = dst_df["episode_index"] == ep
        s = np.stack(dst_df.loc[m, "observation.state"].values)
        a = np.stack(dst_df.loc[m, "action"].values)
        if np.dot(s[0, 3:7], a[0, 3:7]) < 0:
            mis += 1
    print(f"episode 首帧 action vs obs 符号不一致: {mis}/100")

    print("\n=== episode 间 (首帧) ===")
    first_r = []
    first_q_obs = []
    for ep in sorted(src_df["episode_index"].unique()):
        m = src_df["episode_index"] == ep
        s = np.stack(src_df.loc[m, "observation.state"].values)
        first_r.append(s[0, 3])
        first_q_obs.append(np.stack(dst_df.loc[m, "observation.state"].values)[0, 3:7])
    first_r = np.array(first_r)
    first_q_obs = np.array(first_q_obs)
    print(
        f"原 Euler ep0 ori_r: min={first_r.min():.3f} max={first_r.max():.3f} std={first_r.std():.3f}"
    )
    print(f"  ori_r>0: {(first_r > 0).sum()}/100  ori_r<0: {(first_r < 0).sum()}/100")
    dots = []
    for i in range(1, len(first_q_obs)):
        dots.append(np.dot(first_q_obs[i], first_q_obs[i - 1]))
    print(
        "迁移后 ep0 quat: 相邻 episode 首帧 dot<0 仅表示不同起始姿态/半球，"
        "未做跨 episode 全局对齐（设计如此）"
    )

    act = np.stack(src_df["action"].values)
    st = np.stack(src_df["observation.state"].values)
    geo = np.degrees(
        (
            Rotation.from_euler("xyz", act[:, 3:6]).inv()
            * Rotation.from_euler("xyz", st[:, 3:6])
        ).magnitude()
    )
    print("\n=== 同帧 action vs state (原 Euler) ===")
    print(f"geodesic mean={geo.mean():.2f}° p95={np.percentile(geo, 95):.2f}° max={geo.max():.2f}°")
    print(f">30°: {(geo > 30).sum()} ({100 * (geo > 30).mean():.2f}%)")


if __name__ == "__main__":
    main()
