#!/usr/bin/env python3
"""Audit timestamped samples exported from MCAP or synthetic JSONL.

For now this CLI consumes JSONL TopicSample records. Once `rosbag2_py` or an
MCAP Python reader is available in the robot environment, the reader can feed
the same `TopicSample` objects into this script's audit path.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dexterous_platform.audit import audit_samples
from dexterous_platform.defaults import DEFAULT_POLICY_SYNC_WINDOW, DEFAULT_TOPIC_SPECS
from dexterous_platform.io import read_jsonl_samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples-jsonl", type=Path, required=True)
    parser.add_argument("--target-topic", default="/policy/action_filtered")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    samples = read_jsonl_samples(args.samples_jsonl)
    report = audit_samples(
        samples,
        specs=DEFAULT_TOPIC_SPECS,
        sync_windows=(DEFAULT_POLICY_SYNC_WINDOW,),
        target_topic=args.target_topic,
    )
    print(json.dumps(report.to_dict(), indent=2, ensure_ascii=False))
    raise SystemExit(0 if report.passed else 1)


if __name__ == "__main__":
    main()
