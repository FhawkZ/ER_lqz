#!/usr/bin/env python3
"""Merge local LeRobot v3 datasets into one dataset.

Examples:
    python scripts/merge_datasets.py
    python scripts/merge_datasets.py --repo-ids redcube1 redcube2 redcube3 redcube4
    python scripts/merge_datasets.py --repo-ids redcube1 redcube5 --output-repo-id redcube_subset
    python scripts/merge_datasets.py --dataset-root /path/to/data --output-dir /path/to/out
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from lerobot.datasets.dataset_tools import merge_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def _default_dataset_root() -> Path:
    return Path(__file__).resolve().parent.parent / "data"


def _default_repo_ids() -> list[str]:
    return [f"redcube{i}" for i in range(1, 5)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge local LeRobot datasets with consistent schema.")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=_default_dataset_root(),
        help="Parent directory containing source dataset folders (default: <ER_LQZ_ROOT>/data).",
    )
    parser.add_argument(
        "--repo-ids",
        nargs="+",
        default=_default_repo_ids(),
        help="Source dataset folder names under --dataset-root.",
    )
    parser.add_argument(
        "--output-repo-id",
        type=str,
        default="redcube_merged",
        help="Merged dataset id / output folder name.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Explicit output directory (default: <dataset-root>/<output-repo-id>).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove existing output directory before merging.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    dataset_root = args.dataset_root.resolve()
    if not dataset_root.exists():
        raise FileNotFoundError(f"dataset root not found: {dataset_root}")

    missing = [repo_id for repo_id in args.repo_ids if not (dataset_root / repo_id).exists()]
    if missing:
        raise FileNotFoundError(f"missing dataset folders under {dataset_root}: {missing}")

    output_dir = args.output_dir.resolve() if args.output_dir else (dataset_root / args.output_repo_id)
    if output_dir.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"output already exists: {output_dir}\n"
                "Use --overwrite to replace it, or set OUTPUT_REPO_ID / --output-dir."
            )
        shutil.rmtree(output_dir)

    print(f"Merging {len(args.repo_ids)} datasets from {dataset_root}")
    for repo_id in args.repo_ids:
        info_path = dataset_root / repo_id / "meta" / "info.json"
        print(f"  - {repo_id} ({info_path})")

    datasets = [LeRobotDataset(repo_id, root=dataset_root / repo_id) for repo_id in args.repo_ids]
    merged = merge_datasets(
        datasets=datasets,
        output_repo_id=args.output_repo_id,
        output_dir=output_dir,
    )

    print("Merge done.")
    print(f"Output: {output_dir}")
    print(f"Episodes: {merged.meta.total_episodes}")
    print(f"Frames: {merged.meta.total_frames}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
