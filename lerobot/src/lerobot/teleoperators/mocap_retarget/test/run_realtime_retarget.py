#!/usr/bin/env python3
"""MCP 实时 -> 右手局部系 -> dex 解算；可选打印 0–255 指令。

示例（在 dex_retargeting 目录下）::

    python -m mocap_retarget.test.run_realtime_retarget
    python -m mocap_retarget.test.run_realtime_retarget --cmd255 --control-hz 30
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from ..src.dex_retargeter import DexRetargeter, default_dex_paths
from ..src.linker_joint_cmd import rad_to_cmd255
from ..src.mocap_reader import MocapReader
from ..src.utils import frame_to_dex_joint_pos


def main() -> None:
    dex_repo, urdf_root, dex_cfg = default_dex_paths()
    ap = argparse.ArgumentParser(description="MCP 实时重定向 Linker L6。")
    ap.add_argument("--udp-port", type=int, default=7012)
    ap.add_argument("--bvh-rotation", type=int, default=0)
    ap.add_argument("--poll-hz", type=float, default=120.0)
    ap.add_argument("--timeout-s", type=float, default=2.0)
    ap.add_argument("--hand-side", choices=("right", "left"), default="right")
    ap.add_argument("--position-scale", type=float, default=0.01)
    ap.add_argument("--control-hz", type=float, default=30.0)
    ap.add_argument("--print-every", type=int, default=30)
    ap.add_argument("--cmd255", action="store_true", help="同时打印 0–255 关节指令")
    ap.add_argument("--dex-repo", type=Path, default=dex_repo)
    ap.add_argument("--urdf-root", type=Path, default=urdf_root)
    ap.add_argument("--dex-config", type=Path, default=dex_cfg)
    args = ap.parse_args()

    rt = DexRetargeter(args.dex_repo, args.urdf_root, args.dex_config)
    names = rt.target_joint_names
    reader = MocapReader(
        udp_port=args.udp_port,
        bvh_rotation=args.bvh_rotation,
        poll_hz=args.poll_hz,
    )
    period = 1.0 / max(args.control_hz, 1.0)
    n = 0

    reader.open()
    try:
        while True:
            t0 = time.perf_counter()
            frame = reader.read_frame(timeout_s=args.timeout_s)
            joint_pos = frame_to_dex_joint_pos(
                reader, frame, hand_side=args.hand_side, position_scale=args.position_scale
            )
            ref, robot_qpos = rt.retarget(joint_pos)
            q = rt.target_qpos(robot_qpos)

            n += 1
            if n % max(args.print_every, 1) == 0:
                msg = (
                    f"[{n}] q(rad)={np.array2string(q, precision=3, suppress_small=True)} "
                    f"ref={np.array2string(ref, precision=3, suppress_small=True)}"
                )
                if args.cmd255:
                    cmd = rad_to_cmd255(q, names)
                    msg += "  cmd255=" + " ".join(f"{names[i]}={cmd[i]}" for i in range(len(names)))
                print(msg)

            remain = period - (time.perf_counter() - t0)
            if remain > 0:
                time.sleep(remain)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
