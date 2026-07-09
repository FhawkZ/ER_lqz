#!/usr/bin/env python3
"""Slice ``observation.state`` to a selected name subset (LeRobot v3).

Typical use: project heterogeneous fr3_eef datasets onto their state-key
intersection (e.g. 7 joint pos + 6 hand + 7 joint torque = 20D), then merge.

Examples::

    # Slice one dataset
    python scripts/slice_obs_state_keys.py \\
        --input-dir data/joint_assemble_20260708 \\
        --output-dir data/joint_assemble_20260708_s20 \\
        --keep-state \\
            fr3_joint1.pos fr3_joint2.pos fr3_joint3.pos fr3_joint4.pos \\
            fr3_joint5.pos fr3_joint6.pos fr3_joint7.pos \\
            hand_0.pos hand_1.pos hand_2.pos hand_3.pos hand_4.pos hand_5.pos \\
            fr3_joint1.torque fr3_joint2.torque fr3_joint3.torque fr3_joint4.torque \\
            fr3_joint5.torque fr3_joint6.torque fr3_joint7.torque

    # Slice several, writing <output-root>/<name><suffix>
    python scripts/slice_obs_state_keys.py \\
        --input-dirs data/joint_assemble_20260708_old data/joint_assemble_20260708 \\
        --output-root data \\
        --suffix _s20 \\
        --keep-intersection
"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from tqdm import tqdm

_REPO_ROOT = Path(__file__).resolve().parents[1]
_LEROBOT_SRC = _REPO_ROOT / "lerobot" / "src"
if str(_LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(_LEROBOT_SRC))

from lerobot.datasets.compute_stats import aggregate_stats, compute_episode_stats
from lerobot.datasets.io_utils import write_info, write_stats
from lerobot.datasets.utils import EPISODES_DIR, flatten_dict

logger = logging.getLogger(__name__)

DEFAULT_JOINT_ASSEMBLE_INTERSECTION = (
    [f"fr3_joint{i}.pos" for i in range(1, 8)]
    + [f"hand_{i}.pos" for i in range(6)]
    + [f"fr3_joint{i}.torque" for i in range(1, 8)]
)


def _load_info(root: Path) -> dict:
    with open(root / "meta" / "info.json") as f:
        return json.load(f)


def _state_names(info: dict) -> list[str]:
    names = info.get("features", {}).get("observation.state", {}).get("names")
    if not names:
        raise ValueError("observation.state.names missing in info.json")
    return list(names)


def intersection_state_names(roots: list[Path]) -> list[str]:
    """Preserve order from the first dataset; require the same set for all."""
    if not roots:
        raise ValueError("No datasets provided")
    common: set[str] | None = None
    first: list[str] = []
    for root in roots:
        names = _state_names(_load_info(root))
        if not first:
            first = names
        s = set(names)
        common = s if common is None else (common & s)
    assert common is not None
    ordered = [n for n in first if n in common]
    missing_order_check = common - set(ordered)
    if missing_order_check:
        # names only in later datasets; append stably
        ordered.extend(sorted(missing_order_check))
    return ordered


def _copy_tree(src: Path, dst: Path, symlink_videos: bool) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    for name in ("meta", "data", "videos"):
        s = src / name
        if not s.exists():
            continue
        d = dst / name
        if d.exists():
            if d.is_symlink() or d.is_file():
                d.unlink()
            else:
                shutil.rmtree(d)
        if name == "videos" and symlink_videos:
            d.symlink_to(s.resolve())
        else:
            shutil.copytree(s, d)


def _update_info(info: dict, keep_names: list[str]) -> dict:
    info = json.loads(json.dumps(info))
    feat = info["features"]["observation.state"]
    feat["names"] = list(keep_names)
    feat["shape"] = [len(keep_names)]
    return info


def _slice_state_array(states: np.ndarray, src_names: list[str], keep_names: list[str]) -> np.ndarray:
    name_to_idx = {n: i for i, n in enumerate(src_names)}
    missing = [n for n in keep_names if n not in name_to_idx]
    if missing:
        raise KeyError(f"Missing state keys in source: {missing}")
    idx = [name_to_idx[n] for n in keep_names]
    return np.asarray(states, dtype=np.float32)[:, idx]


def _process_data_parquet(
    src_path: Path,
    dst_path: Path,
    src_names: list[str],
    keep_names: list[str],
) -> dict[int, dict]:
    df = pd.read_parquet(src_path)
    episode_stats_inputs: dict[int, dict] = {}

    for ep in sorted(df["episode_index"].unique()):
        mask = df["episode_index"] == ep
        states = np.stack(df.loc[mask, "observation.state"].to_numpy())
        actions = np.stack(df.loc[mask, "action"].to_numpy())
        sliced = _slice_state_array(states, src_names, keep_names)

        row_index = df.index[mask]
        df.loc[mask, "observation.state"] = pd.Series(
            [sliced[i].astype(np.float32) for i in range(len(sliced))],
            index=row_index,
        )
        episode_stats_inputs[int(ep)] = {
            "action": np.asarray(actions, dtype=np.float32),
            "observation.state": sliced.astype(np.float32),
        }

    dst_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(dst_path, index=False)
    return episode_stats_inputs


def _rebuild_episodes_meta(
    src_root: Path,
    dst_root: Path,
    info: dict,
    episode_stats_by_ep: dict[int, dict],
) -> None:
    numeric_features = {
        k: v
        for k, v in info["features"].items()
        if v["dtype"] not in ("image", "video", "string")
        and k not in {"timestamp", "frame_index", "episode_index", "index", "task_index"}
    }

    src_ep_files = sorted((src_root / EPISODES_DIR).glob("*/*.parquet"))
    if not src_ep_files:
        raise FileNotFoundError(f"No episodes parquet under {src_root / EPISODES_DIR}")

    dst_ep_dir = dst_root / EPISODES_DIR
    if dst_ep_dir.exists():
        shutil.rmtree(dst_ep_dir)

    all_episode_stats: list[dict] = []

    for src_ep_path in src_ep_files:
        ep_df = pd.read_parquet(src_ep_path)
        non_stats_cols = [c for c in ep_df.columns if not c.startswith("stats/")]
        base_df = ep_df[non_stats_cols].copy()

        new_rows = []
        for row_idx in range(len(base_df)):
            row = base_df.iloc[row_idx].to_dict()
            ep_idx = int(row["episode_index"])
            if ep_idx not in episode_stats_by_ep:
                raise KeyError(f"Missing transformed data for episode {ep_idx}")

            ep_data = episode_stats_by_ep[ep_idx]
            ep_stats = compute_episode_stats(ep_data, numeric_features)
            all_episode_stats.append(ep_stats)
            row.update(flatten_dict({"stats": ep_stats}))
            new_rows.append(row)

        out_df = pd.DataFrame(new_rows)
        rel = src_ep_path.relative_to(src_root / EPISODES_DIR)
        dst_path = dst_ep_dir / rel
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        out_df.to_parquet(dst_path, index=False)

    global_stats = aggregate_stats(all_episode_stats)
    src_stats_path = src_root / "meta" / "stats.json"
    if src_stats_path.exists():
        with open(src_stats_path) as f:
            src_stats = json.load(f)
        for key, value in src_stats.items():
            if key not in global_stats and key != "observation.state":
                global_stats[key] = value

    write_stats(global_stats, dst_root)


def slice_dataset(
    src_root: Path,
    dst_root: Path,
    keep_names: list[str],
    *,
    symlink_videos: bool = True,
    overwrite: bool = False,
) -> None:
    src_root = src_root.resolve()
    dst_root = dst_root.resolve()

    if dst_root.exists():
        if not overwrite:
            raise FileExistsError(f"Output exists: {dst_root} (use --overwrite)")
        if dst_root.is_symlink() or dst_root.is_file():
            dst_root.unlink()
        else:
            shutil.rmtree(dst_root)

    info = _load_info(src_root)
    src_names = _state_names(info)
    missing = [n for n in keep_names if n not in src_names]
    if missing:
        raise KeyError(f"{src_root.name}: missing keep-state keys: {missing}")

    logger.info(
        "Slicing %s -> %s  state %dD -> %dD",
        src_root,
        dst_root,
        len(src_names),
        len(keep_names),
    )
    _copy_tree(src_root, dst_root, symlink_videos=symlink_videos)

    info = _update_info(info, keep_names)
    write_info(info, dst_root)

    src_data_files = sorted((src_root / "data").glob("*/*.parquet"))
    episode_stats_by_ep: dict[int, dict] = {}

    for src_pq in tqdm(src_data_files, desc=f"data {src_root.name}"):
        rel = src_pq.relative_to(src_root / "data")
        dst_pq = dst_root / "data" / rel
        ep_partial = _process_data_parquet(src_pq, dst_pq, src_names, keep_names)
        for ep_idx, data in ep_partial.items():
            if ep_idx in episode_stats_by_ep:
                raise RuntimeError(f"Duplicate episode_index {ep_idx} across data files")
            episode_stats_by_ep[ep_idx] = data

    _rebuild_episodes_meta(src_root, dst_root, info, episode_stats_by_ep)
    logger.info("Done: %s (episodes=%d, state=%dD)", dst_root, len(episode_stats_by_ep), len(keep_names))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--input-dir", type=Path, help="Single source dataset root")
    src.add_argument("--input-dirs", nargs="+", type=Path, help="Multiple source dataset roots")

    p.add_argument("--output-dir", type=Path, help="Output root (with --input-dir)")
    p.add_argument(
        "--output-root",
        type=Path,
        help="Parent for outputs (with --input-dirs); writes <output-root>/<name><suffix>",
    )
    p.add_argument("--suffix", type=str, default="_s20", help="Suffix for multi-input outputs")

    keep = p.add_mutually_exclusive_group()
    keep.add_argument(
        "--keep-state",
        nargs="+",
        default=None,
        help="Exact observation.state names to keep (ordered)",
    )
    keep.add_argument(
        "--keep-intersection",
        action="store_true",
        help="Keep intersection of observation.state names across --input-dirs "
        f"(default ordered keys for joint_assemble: {len(DEFAULT_JOINT_ASSEMBLE_INTERSECTION)}D)",
    )

    p.add_argument("--symlink-videos", action="store_true", default=True)
    p.add_argument("--copy-videos", action="store_true", help="Copy videos instead of symlink")
    p.add_argument("--overwrite", action="store_true")
    return p.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = parse_args()
    symlink_videos = not args.copy_videos

    if args.input_dir is not None:
        inputs = [args.input_dir]
        if args.output_dir is None:
            raise SystemExit("--output-dir is required with --input-dir")
        outputs = [args.output_dir]
    else:
        inputs = list(args.input_dirs)
        if args.output_root is None:
            raise SystemExit("--output-root is required with --input-dirs")
        outputs = [args.output_root / f"{p.name}{args.suffix}" for p in inputs]

    if args.keep_state:
        keep_names = list(args.keep_state)
    elif args.keep_intersection:
        keep_names = intersection_state_names(inputs)
        if not keep_names:
            raise SystemExit("Empty state-key intersection")
        logger.info("Intersection (%dD): %s", len(keep_names), keep_names)
    else:
        # Safe default for the two joint_assemble datasets.
        keep_names = list(DEFAULT_JOINT_ASSEMBLE_INTERSECTION)
        logger.info("Using default joint_assemble intersection (%dD)", len(keep_names))

    for src, dst in zip(inputs, outputs):
        slice_dataset(
            src,
            dst,
            keep_names,
            symlink_videos=symlink_videos,
            overwrite=args.overwrite,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
