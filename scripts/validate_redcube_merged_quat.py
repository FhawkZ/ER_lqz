#!/usr/bin/env python3
"""Validate fr3_eef Euler->quat migration output."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

SRC = Path("/media/data/liqz/data/redcube_merged")
DST = Path("/media/data/liqz/data/redcube_merged_quat")


def main() -> int:
    issues: list[str] = []
    warnings: list[str] = []

    def ok(msg: str) -> None:
        print(f"OK   {msg}")

    def fail(msg: str) -> None:
        issues.append(msg)
        print(f"FAIL {msg}")

    def warn(msg: str) -> None:
        warnings.append(msg)
        print(f"WARN {msg}")

    with open(SRC / "meta/info.json") as f:
        src_info = json.load(f)
    with open(DST / "meta/info.json") as f:
        dst_info = json.load(f)

    if dst_info["total_episodes"] != src_info["total_episodes"]:
        fail(f"total_episodes {dst_info['total_episodes']} != {src_info['total_episodes']}")
    else:
        ok(f"total_episodes={dst_info['total_episodes']}")

    if dst_info["total_frames"] != src_info["total_frames"]:
        fail(f"total_frames {dst_info['total_frames']} != {src_info['total_frames']}")
    else:
        ok(f"total_frames={dst_info['total_frames']}")

    for key in ("action", "observation.state"):
        sh = dst_info["features"][key]["shape"]
        names = dst_info["features"][key]["names"]
        if sh != [13] or "ori_qx.pos" not in names or "ori_r.pos" in names:
            fail(f"{key} bad schema shape={sh} names={names}")
        else:
            ok(f"{key}: 13D quaternion schema")

    src_df = pd.read_parquet(next((SRC / "data").glob("*/*.parquet")))
    dst_df = pd.read_parquet(next((DST / "data").glob("*/*.parquet")))
    if len(src_df) != len(dst_df):
        fail(f"rows {len(dst_df)} != {len(src_df)}")
    else:
        ok(f"data rows={len(dst_df)}")

    quat_sl = slice(3, 7)
    for ep in range(int(dst_info["total_episodes"])):
        m = dst_df["episode_index"] == ep
        acts = np.stack(dst_df.loc[m, "action"].values)
        states = np.stack(dst_df.loc[m, "observation.state"].values)
        if acts.shape[1] != 13 or states.shape[1] != 13:
            fail(f"ep{ep} wrong dim")
            break
        for label, arr in (("action", acts), ("state", states)):
            norms = np.linalg.norm(arr[:, quat_sl], axis=1)
            if np.any(np.abs(norms - 1.0) > 1e-3):
                fail(f"ep{ep} {label} quat norm max_err={np.max(np.abs(norms - 1)):.4f}")
                break
    else:
        ok("all episodes: dim=13, quat unit norm")

    bad_obs = bad_act = 0
    for ep in range(int(dst_info["total_episodes"])):
        m = dst_df["episode_index"] == ep
        states = np.stack(dst_df.loc[m, "observation.state"].values)
        acts = np.stack(dst_df.loc[m, "action"].values)
        for i in range(1, len(states)):
            if np.dot(states[i, 3:7], states[i - 1, 3:7]) < 0:
                bad_obs += 1
            if np.dot(acts[i, 3:7], acts[i - 1, 3:7]) < 0:
                bad_act += 1
    if bad_obs:
        fail(f"obs hemisphere breaks={bad_obs}")
    else:
        ok("obs quaternion sign continuous")
    if bad_act:
        fail(f"action hemisphere breaks={bad_act}")
    else:
        ok("action quaternion sign continuous")

    misalign = 0
    for ep in range(int(dst_info["total_episodes"])):
        m = dst_df["episode_index"] == ep
        states = np.stack(dst_df.loc[m, "observation.state"].values)
        acts = np.stack(dst_df.loc[m, "action"].values)
        if np.dot(states[0, 3:7], acts[0, 3:7]) < 0:
            misalign += 1
    if misalign:
        fail(f"action[0] vs obs[0] misaligned in {misalign} episodes")
    else:
        ok("action[0] aligned to obs[0] in all episodes")

    max_pos_err = max_hand_err = 0.0
    for i in range(min(1000, len(src_df))):
        sa = np.asarray(src_df.iloc[i]["action"], dtype=np.float64)
        ss = np.asarray(src_df.iloc[i]["observation.state"], dtype=np.float64)
        da = np.asarray(dst_df.iloc[i]["action"], dtype=np.float64)
        ds = np.asarray(dst_df.iloc[i]["observation.state"], dtype=np.float64)
        max_pos_err = max(
            max_pos_err,
            np.max(np.abs(da[:3] - sa[:3])),
            np.max(np.abs(ds[:3] - ss[:3])),
        )
        max_hand_err = max(
            max_hand_err,
            np.max(np.abs(da[7:] - sa[6:])),
            np.max(np.abs(ds[7:] - ss[6:])),
        )
    if max_pos_err > 1e-5 or max_hand_err > 1e-5:
        fail(f"xyz/hand drift max_pos={max_pos_err} max_hand={max_hand_err}")
    else:
        ok(f"xyz/hand unchanged (1k frames)")

    for cam in ("observation.images.handeye", "observation.images.fixed"):
        vp = DST / "videos" / cam
        if not vp.exists():
            fail(f"missing {vp}")
        elif vp.is_symlink():
            ok(f"{cam} symlink ok")
        else:
            warn(f"{cam} not symlink (copied)")

    with open(DST / "meta/stats.json") as f:
        stats = json.load(f)
    for key in ("action", "observation.state"):
        if len(stats[key].get("mean", [])) != 13:
            fail(f"stats[{key}] dim != 13")
    if not any("stats[" in x for x in issues):
        ok("meta/stats.json 13D stats")

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lerobot" / "src"))
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        ds = LeRobotDataset(repo_id="redcube_merged_quat", root=str(DST))
        item = ds[0]
        if item["action"].shape[-1] == 13 and item["observation.state"].shape[-1] == 13:
            ok("LeRobotDataset smoke load")
        else:
            fail("LeRobotDataset bad tensor shapes")
    except Exception as exc:
        warn(f"LeRobotDataset: {exc}")

    print("\n=== Summary ===")
    print(f"issues={len(issues)} warnings={len(warnings)}")
    for x in issues:
        print(f"  - {x}")
    for x in warnings:
        print(f"  - {x}")
    return 1 if issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
