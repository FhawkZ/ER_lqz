#!/usr/bin/env python3
"""Analyze redcube1 dataset for Euler ±pi branch jumps."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[1] / "data" / "redcube1"


def report(label: str, ori: np.ndarray, ep: np.ndarray) -> None:
    print(f"\n{'=' * 60}\n{label}\n{'=' * 60}")
    labs = ["ori_r", "ori_p", "ori_y"]
    for j, lab in enumerate(labs):
        c = ori[:, j]
        print(
            f"{lab:6s}: min={c.min():7.3f} max={c.max():7.3f} mean={c.mean():7.3f} "
            f"|c|>2.5: {(np.abs(c) > 2.5).sum():5d} ({100 * np.mean(np.abs(c) > 2.5):5.2f}%)"
        )

    dr_big = dn_big = n_trans = 0
    first_r: list[float] = []
    ep_jump_examples: list[tuple] = []
    for e in np.unique(ep):
        o = ori[ep == e]
        if len(o) < 2:
            continue
        n_trans += len(o) - 1
        dr = np.abs(np.diff(o[:, 0]))
        dn = np.linalg.norm(np.diff(o, axis=0), axis=1)
        dr_big += int((dr > 2.0).sum())
        dn_big += int((dn > 2.0).sum())
        first_r.append(float(o[0, 0]))
        for k in np.where(dr > 2.0)[0][:2]:
            if len(ep_jump_examples) < 8:
                ep_jump_examples.append((int(e), int(k), float(o[k, 0]), float(o[k + 1, 0]), float(dr[k])))

    print(
        f"within-episode |Δori_r|>2.0 rad: {dr_big}/{n_trans} ({100 * dr_big / n_trans:.2f}%)"
    )
    print(
        f"within-episode |Δeuler|>2.0 rad: {dn_big}/{n_trans} ({100 * dn_big / n_trans:.2f}%)"
    )
    first_r_arr = np.array(first_r)
    print(
        f"episode start ori_r: min={first_r_arr.min():.3f} max={first_r_arr.max():.3f} "
        f"std={first_r_arr.std():.3f}"
    )
    print(
        f"  start ori_r>0: {(first_r_arr > 0).sum()}/{len(first_r_arr)}  "
        f"start ori_r<0: {(first_r_arr < 0).sum()}/{len(first_r_arr)}"
    )
    if ep_jump_examples:
        print("  example ori_r jumps (ep, frame, r_k, r_k+1, |dr|):")
        for ex in ep_jump_examples:
            print(f"    ep={ex[0]} f{ex[1]}->{ex[1]+1}: {ex[2]:.3f} -> {ex[3]:.3f}  |dr|={ex[4]:.3f}")

    q_flip = q_total = 0
    qw_neg = 0
    for e in np.unique(ep):
        o = ori[ep == e]
        q = R.from_euler("xyz", o).as_quat()
        qw_neg += int((q[:, 3] < 0).sum())
        if len(q) < 2:
            continue
        dots = np.sum(q[1:] * q[:-1], axis=1)
        q_flip += int((dots < 0).sum())
        q_total += len(dots)
    print(
        f"frames with qw<0 (if converted): {qw_neg}/{len(ori)} "
        f"({100 * qw_neg / len(ori):.2f}%)"
    )
    print(
        f"within-episode quat sign flip (dot<0): {q_flip}/{q_total} "
        f"({100 * q_flip / max(q_total, 1):.2f}%)"
    )


def main() -> None:
    info = json.loads((ROOT / "meta/info.json").read_text())
    parquets = sorted((ROOT / "data").rglob("*.parquet"))
    df = pd.concat([pd.read_parquet(p) for p in parquets], ignore_index=True)

    names = info["features"]["action"]["names"]
    act = np.stack(df["action"].values, axis=0).astype(np.float64)
    state = np.stack(df["observation.state"].values, axis=0).astype(np.float64)
    ep = df["episode_index"].to_numpy()

    print(
        f"Dataset: {ROOT.name}  robot={info['robot_type']}  "
        f"episodes={info['total_episodes']}  frames={len(df)}  repr=Euler ori_r/p/y"
    )

    idx = {n: i for i, n in enumerate(names)}
    ori_i = [idx["ori_r.pos"], idx["ori_p.pos"], idx["ori_y.pos"]]

    report("ACTION", act[:, ori_i], ep)
    report("OBSERVATION.STATE", state[:, ori_i], ep)

    act_o = act[:, ori_i]
    st_o = state[:, ori_i]
    geo = np.degrees(
        (R.from_euler("xyz", act_o).inv() * R.from_euler("xyz", st_o)).magnitude()
    )
    print(f"\n{'=' * 60}\naction vs state (geodesic deg, same frame)\n{'=' * 60}")
    print(
        f"mean={geo.mean():.2f}  p95={np.percentile(geo, 95):.2f}  max={geo.max():.2f}"
    )
    print(
        f">30°: {(geo > 30).sum()} ({100 * (geo > 30).mean():.2f}%)  "
        f">90°: {(geo > 90).sum()} ({100 * (geo > 90).mean():.2f}%)"
    )

    print(f"\n{'=' * 60}\nepisode boundaries: action ori_r euler diff vs geodesic\n{'=' * 60}")
    big_euler_small_geo = 0
    for e in sorted(np.unique(ep))[:-1]:
        end_euler = act[ep == e][-1, ori_i]
        start_euler = act[ep == e + 1][0, ori_i]
        dr = abs(float(start_euler[0] - end_euler[0]))
        g = float(
            np.degrees(
                (
                    R.from_euler("xyz", end_euler).inv()
                    * R.from_euler("xyz", start_euler)
                ).magnitude()
            )
        )
        if dr > 2.0:
            print(
                f"  ep {int(e)}->{int(e)+1}: Δori_r={dr:.3f} rad  geodesic={g:.1f}°  "
                f"(end_r={end_euler[0]:.3f} start_r={start_euler[0]:.3f})"
            )
            if g < 30.0:
                big_euler_small_geo += 1
    print(f"boundaries with |Δori_r|>2 but geodesic<30° (likely ±π artifact): {big_euler_small_geo}")


if __name__ == "__main__":
    main()
