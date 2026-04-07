#!/usr/bin/env python3
"""Merge local LeRobot datasets (redcube1..redcube14) into one dataset.

Usage:
    python scripts/merge_redcube_datasets.py
    python scripts/merge_redcube_datasets.py --output-repo-id redcube_merged_v2
    python scripts/merge_redcube_datasets.py --dataset-root /path/to/Data/franka_hand
"""

from __future__ import annotations

import argparse
from pathlib import Path

from lerobot.datasets.dataset_tools import merge_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def build_default_repo_ids() -> list[str]:
    return [f"redcube{i}" for i in range(1, 15)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge multiple local LeRobot datasets with consistent schema."
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/home/franka/lqz/Data/franka_hand"),
        help="Parent directory that contains redcube1..redcube14 folders.",
    )
    parser.add_argument(
        "--repo-ids",
        nargs="+",
        default=build_default_repo_ids(),
        help="Dataset folder names under --dataset-root.",
    )
    parser.add_argument(
        "--output-repo-id",
        type=str,
        default="redcube_merged",
        help="Merged dataset id/folder name.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional explicit output directory. "
        "Default: <dataset-root>/<output-repo-id>.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    dataset_root = args.dataset_root.resolve()
    if not dataset_root.exists():
        raise FileNotFoundError(f"dataset root not found: {dataset_root}")

    missing = [repo_id for repo_id in args.repo_ids if not (dataset_root / repo_id).exists()]
    if missing:
        raise FileNotFoundError(
            f"missing dataset folders under {dataset_root}: {missing}"
        )

    # LeRobotDataset expects `root` to be the dataset directory itself.
    # If we pass the parent directory, it may fallback to Hub download.
    datasets = [LeRobotDataset(repo_id, root=dataset_root / repo_id) for repo_id in args.repo_ids]
    output_dir = args.output_dir.resolve() if args.output_dir else (dataset_root / args.output_repo_id)

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
    main()
