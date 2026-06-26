#!/usr/bin/env python3
"""Run a synthetic end-to-end smoke test for the dexterous platform package."""

from __future__ import annotations

import json
import sys
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dexterous_platform.audit import audit_samples
from dexterous_platform.converters import build_frames
from dexterous_platform.defaults import DEFAULT_POLICY_SYNC_WINDOW
from dexterous_platform.recording import RecordingManifest, build_rosbag2_command
from dexterous_platform.schema import ConversionPlan, ConversionTopic, LookupRule, TopicSample, TopicSpec


def sample(topic: str, t: float, payload: dict | None = None, seq: int | None = None) -> TopicSample:
    return TopicSample(
        topic=topic,
        source_time=t,
        publish_time=t + 0.001,
        receive_time=t + 0.004,
        sequence_id=seq,
        payload=payload or {},
    )


def main() -> None:
    samples = []
    for i in range(20):
        samples.append(sample("/franka/ee_pose", i * 0.01, {"x": i * 0.001}, i))
    for i in range(10):
        t = i * 0.02
        samples.append(sample("/linkerhand/right/joint_states", t, {"hand_0": i}, i))
        samples.append(sample("/camera/handeye/color/image_rect_raw", t, {"frame": i}, i))
        samples.append(sample("/camera/fixed/color/image_rect_raw", t, {"frame": i}, i))
        samples.append(sample("/policy/action_filtered", t, {"ee_x": i * 0.01}, i))

    report = audit_samples(
        samples,
        specs=(
            TopicSpec("/franka/ee_pose", nominal_hz=100, max_p99_period_s=0.02, max_p99_receive_latency_s=0.02),
            TopicSpec("/linkerhand/right/joint_states", nominal_hz=50, max_p99_period_s=0.04, max_p99_receive_latency_s=0.03),
            TopicSpec("/camera/handeye/color/image_rect_raw", nominal_hz=50, max_p99_period_s=0.04, max_p99_receive_latency_s=0.08),
            TopicSpec("/camera/fixed/color/image_rect_raw", nominal_hz=50, max_p99_period_s=0.04, max_p99_receive_latency_s=0.08),
            TopicSpec("/policy/action_filtered", nominal_hz=50, max_p99_period_s=0.04, max_p99_receive_latency_s=0.02),
        ),
        sync_windows=(DEFAULT_POLICY_SYNC_WINDOW,),
        target_topic="/policy/action_filtered",
    )

    plan = ConversionPlan(
        target_topic="/policy/action_filtered",
        task="synthetic pick",
        topics=(
            ConversionTopic("/franka/ee_pose", "observation.arm", LookupRule.INTERPOLATE, max_age_s=0.001),
            ConversionTopic("/linkerhand/right/joint_states", "observation.hand", LookupRule.PREVIOUS, max_age_s=0.03),
            ConversionTopic("/camera/handeye/color/image_rect_raw", "observation.image", LookupRule.PREVIOUS, max_age_s=0.08),
            ConversionTopic("/policy/action_filtered", "action", LookupRule.NEAREST, max_age_s=0.001),
        ),
    )
    frames = build_frames(samples, plan)

    manifest = RecordingManifest(
        recording_id="synthetic_trial_001",
        task="synthetic pick",
        output_dir=Path("/tmp/dexterous_records"),
        topics=(
            "/franka/ee_pose",
            "/linkerhand/right/joint_states",
            "/camera/handeye/color/image_rect_raw",
            "/camera/fixed/color/image_rect_raw",
            "/policy/action_filtered",
        ),
    )

    result = {
        "audit_passed": report.passed,
        "audit_issues": [asdict(issue) for issue in report.issues],
        "frame_count": len(frames),
        "rosbag2_command": build_rosbag2_command(manifest),
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not report.passed or not frames:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
