#!/usr/bin/env python3
"""Plan or execute rosbag2 MCAP recording for the dexterous platform."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dexterous_platform.recording import FR3RecordingSession


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--recording-id", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--include-optional-topics", default="true", choices=["true", "false"])
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = FR3RecordingSession(
        recording_id=args.recording_id,
        task=args.task,
        output_dir=args.output_dir,
        include_optional_topics=args.include_optional_topics == "true",
    )
    command = session.rosbag2_command()
    print(" ".join(command))
    if args.execute:
        raise SystemExit(subprocess.call(command))


if __name__ == "__main__":
    main()
