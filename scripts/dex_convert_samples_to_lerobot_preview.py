#!/usr/bin/env python3
"""Preview MCAP-to-LeRobot timeline alignment using JSONL TopicSample input."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dexterous_platform.converters import build_frames
from dexterous_platform.defaults import DEFAULT_LEROBOT_PLAN
from dexterous_platform.io import read_jsonl_samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples-jsonl", type=Path, required=True)
    parser.add_argument("--task", default="pick the red cube and drop it in box")
    parser.add_argument("--limit", type=int, default=5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    samples = read_jsonl_samples(args.samples_jsonl)
    plan = replace(DEFAULT_LEROBOT_PLAN, task=args.task)
    frames = build_frames(samples, plan)
    preview = {
        "frame_count": len(frames),
        "preview": [
            {
                "timestamp": frame.timestamp,
                "task": frame.task,
                "keys": sorted(frame.values.keys()),
                "sources": {key: asdict(source) for key, source in frame.sources.items()},
            }
            for frame in frames[: args.limit]
        ],
    }
    print(json.dumps(preview, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
