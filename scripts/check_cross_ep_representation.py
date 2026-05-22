#!/usr/bin/env python3
"""Check: same physical pose, different representation across episodes."""

from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

SRC = Path("/media/data/liqz/data/redcube_merged")
DST = Path("/media/data/liqz/data/redcube_merged_quat")


def main() -> None:
    sdf = pd.read_parquet(next((SRC / "data").glob("*/*.parquet")))
    ddf = pd.read_parquet(next((DST / "data").glob("*/*.parquet")))

    ep0_e, ep0_q = [], []
    for ep in sorted(sdf["episode_index"].unique()):
        m = sdf["episode_index"] == ep
        s = np.stack(sdf.loc[m, "observation.state"].values)
        ep0_e.append(s[0, 3:6])
        dq = np.stack(ddf.loc[ddf["episode_index"] == ep, "observation.state"].values)
        ep0_q.append(dq[0, 3:7])

    ep0_e = np.array(ep0_e)
    ep0_q = np.array(ep0_q)
    re = Rotation.from_euler("xyz", ep0_e)
    rq = Rotation.from_quat(ep0_q)

    e_bad = q_bad = 0
    n_close = 0
    for i in range(len(ep0_e)):
        for j in range(i + 1, len(ep0_e)):
            ang = np.degrees((re[i].inv() * re[j]).magnitude())
            if ang >= 15.0:
                continue
            n_close += 1
            if np.linalg.norm(ep0_e[i] - ep0_e[j]) > 1.0:
                e_bad += 1
            if np.dot(ep0_q[i], ep0_q[j]) < 0:
                q_bad += 1

    print("相近物理姿态 (episode 首帧 geodesic < 15°) 的对数:", n_close)
    print(f"  [原 Euler] 欧拉向量差很大 (L2>1 rad，同姿不同表): {e_bad}/{n_close}")
    print(f"  [迁移 Quat] 四元数反号 (dot<0，同姿不同表): {q_bad}/{n_close}")
    print(f"  ep0 中 qw<0 的条数: {(ep0_q[:, 3] < 0).sum()}/{len(ep0_q)}")
    print()
    print("结论:")
    if q_bad == 0 and e_bad > 0:
        print("  - 全局表征统一 OK（相近姿态无 q/-q 混用）")
    elif q_bad > 0:
        print(f"  - 仍有同姿不同表: quat dot<0 对数={q_bad}")
    print("  - episode 内时间连续请用 validate 脚本单独检查")


if __name__ == "__main__":
    main()
