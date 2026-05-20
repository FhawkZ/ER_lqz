#!/usr/bin/env python3
"""调试：间隔打印右手子树相对 RightHand 的 xyz。

示例（在 dex_retargeting 目录下）::

    python -m mocap_retarget.test.read_right_hand_subtree
"""

from __future__ import annotations

import argparse
import time

from ..src.mocap_reader import MocapReader

ROOT = "RightHand"


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--udp-port", type=int, default=7012)
    p.add_argument("--poll-hz", type=float, default=120.0)
    p.add_argument("--timeout-s", type=float, default=2.0)
    p.add_argument("--interval-s", type=float, default=1.0)
    p.add_argument("--position-scale", type=float, default=1.0)
    args = p.parse_args()

    reader = MocapReader(
        udp_port=args.udp_port,
        poll_hz=args.poll_hz,
        position_scale=args.position_scale,
    )
    reader.open()
    n = 0
    try:
        while True:
            t0 = time.perf_counter()
            frame = reader.read_frame(timeout_s=args.timeout_s)
            rel = reader.build_subtree_relative_transforms(frame, ROOT)
            n += 1
            print(f"\n=== frame {n} t={frame.timestamp_s:.3f}s ===")
            for name in sorted(rel):
                x, y, z = rel[name].position
                print(f"  {name:<22} {x:+.4f} {y:+.4f} {z:+.4f}")
            dt = time.perf_counter() - t0
            if args.interval_s > dt:
                time.sleep(args.interval_s - dt)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
