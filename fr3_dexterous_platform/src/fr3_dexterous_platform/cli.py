"""Command line entrypoints."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from fr3_dexterous_platform.adapters.mock import MockFr3LinkerRobot, ScriptedTeleopSource
from fr3_dexterous_platform.datasets.writers import MultiFormatWriter
from fr3_dexterous_platform.inference.commands import openpi_server_command
from fr3_dexterous_platform.inference.remote import PolicyHttpServer, RemotePolicyClient
from fr3_dexterous_platform.inference.runtime import EchoPolicy
from fr3_dexterous_platform.recording.control_loop import collect_episode
from fr3_dexterous_platform.schemas import CollectionConfig, FreshnessSpec
from fr3_dexterous_platform.teleop.policy_source import PolicyActionSource


def main() -> None:
    parser = argparse.ArgumentParser(prog="fr3dex")
    sub = parser.add_subparsers(dest="cmd", required=True)

    collect = sub.add_parser("collect")
    collect.add_argument("--backend", choices=["mock", "native-ros"], default="mock")
    collect.add_argument(
        "--action-source",
        choices=["mock-teleop", "native-mocap", "isaac-udp", "remote-policy"],
        default="mock-teleop",
    )
    collect.add_argument("--server-address", default="http://127.0.0.1:8088")
    collect.add_argument("--isaac-host", default="0.0.0.0")
    collect.add_argument("--isaac-port", type=int, default=15050)
    collect.add_argument("--formats", default="jsonl,lerobot")
    collect.add_argument("--output-dir", type=Path, required=True)
    collect.add_argument("--frames", type=int, default=30)
    collect.add_argument("--duration-s", type=float, default=None)
    collect.add_argument("--fps", type=float, default=30.0)
    collect.add_argument("--task", required=True)
    collect.add_argument("--drop-stale-frames", action="store_true")

    serve = sub.add_parser("serve-policy")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8088)
    serve.add_argument("--policy", choices=["echo"], default="echo")

    plan_openpi = sub.add_parser("plan-openpi-server")
    plan_openpi.add_argument("--host", default="0.0.0.0")
    plan_openpi.add_argument("--port", type=int, default=8090)
    plan_openpi.add_argument("--model-path", required=True)

    args = parser.parse_args()
    if args.cmd == "collect":
        _collect(args)
    elif args.cmd == "serve-policy":
        server = PolicyHttpServer(args.host, args.port, EchoPolicy())
        print(json.dumps({"serving": f"http://{args.host}:{args.port}", "policy": args.policy}, ensure_ascii=False))
        server.serve_forever()
    elif args.cmd == "plan-openpi-server":
        print(json.dumps({"server": openpi_server_command(args.host, args.port, args.model_path)}, indent=2))


def _collect(args: argparse.Namespace) -> None:
    if args.backend == "mock":
        robot = MockFr3LinkerRobot()
    elif args.backend == "native-ros":
        from fr3_dexterous_platform.adapters.native_ros import NativeRosFr3Backend

        robot = NativeRosFr3Backend()

    if args.action_source == "mock-teleop":
        action_source = ScriptedTeleopSource()
    elif args.action_source == "native-mocap":
        from fr3_dexterous_platform.teleop.native_mocap import NativeMocapRetargetSource

        action_source = NativeMocapRetargetSource()
    elif args.action_source == "isaac-udp":
        from fr3_dexterous_platform.isaac.udp_source import IsaacUdpActionSource

        action_source = IsaacUdpActionSource(host=args.isaac_host, port=args.isaac_port)
    elif args.action_source == "remote-policy":
        action_source = PolicyActionSource(RemotePolicyClient(args.server_address))

    writer = MultiFormatWriter.from_formats(args.output_dir, args.formats.split(","))
    freshness = FreshnessSpec(
        required_keys=("arm.ee_pose", "hand.joints", "image.handeye", "image.fixed"),
        max_age_s={
            "arm.ee_pose": 0.03,
            "hand.joints": 0.04,
            "image.handeye": 0.10,
            "image.fixed": 0.10,
        },
    )
    config = CollectionConfig(
        task=args.task,
        fps=args.fps,
        frames=args.frames,
        duration_s=args.duration_s,
        freshness=freshness,
        drop_stale_frames=args.drop_stale_frames,
    )
    result = collect_episode(robot, action_source, writer, config)
    print(
        json.dumps(
            {
                "written_frames": result.written_frames,
                "dropped_frames": result.dropped_frames,
                "issues": result.issues[:20],
                "output_dir": str(args.output_dir),
            },
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
