#!/usr/bin/env python3
"""Migrate LeRobot v3 fr3_eef datasets from Euler EE pose to quaternion + sign alignment.

Targets datasets like ``data/redcube1`` where ``action`` / ``observation.state`` use
``ori_r, ori_p, ori_y``. Output matches live teleop/record:

* Matches live temporal rules (no per-frame action/obs sync): episode 0 ``action`` aligns
  to ``obs``; later frames use ``q_i · q_{i-1} >= 0`` per stream.
* **Default** ``--global-quat-canonical``: offline Euler data need dataset-wide ``q_ref``
  (live record assumes ROS ``current_pose`` is already consistent; migrated data are not).

Also refreshes ``meta/info.json``, ``meta/stats.json``, and ``meta/episodes/*.parquet``
(per-episode stats columns).

Example::

    python scripts/migrate_fr3_eef_euler_to_quat.py \\
        --input-dir data/redcube1 \\
        --output-dir data/redcube1_quat

    # Batch: every child under data/ that looks like an Euler fr3_eef dataset
    python scripts/migrate_fr3_eef_euler_to_quat.py \\
        --input-root data \\
        --output-root data_migrated \\
        --pattern 'redcube*'
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
from scipy.spatial.transform import Rotation
from tqdm import tqdm

# Allow running from repo root without installing lerobot as a package.
_REPO_ROOT = Path(__file__).resolve().parents[1]
_LEROBOT_SRC = _REPO_ROOT / "lerobot" / "src"
if str(_LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(_LEROBOT_SRC))

from lerobot.datasets.compute_stats import aggregate_stats, compute_episode_stats
from lerobot.datasets.io_utils import write_info, write_stats
from lerobot.datasets.utils import (
    DEFAULT_EPISODES_PATH,
    EPISODES_DIR,
    flatten_dict,
)
from lerobot.utils.quat import align_quat_hemisphere, normalize_quat_xyzw, prepare_quat_xyzw

logger = logging.getLogger(__name__)

# Keep in sync with ``FR3EEFConfig`` (avoid importing fr3_eef → rclpy).
FR3_EEF_ARM_POSE_NAMES = (
    "ee_x",
    "ee_y",
    "ee_z",
    "ori_qx",
    "ori_qy",
    "ori_qz",
    "ori_qw",
)
FR3_EEF_HAND_JOINT_NAMES = tuple(f"hand_{i}" for i in range(6))

EULER_ORI_KEYS = ("ori_r.pos", "ori_p.pos", "ori_y.pos")
NUMERIC_KEYS = ("action", "observation.state")


def _is_migratable_fr3_eef(info: dict) -> bool:
    if info.get("robot_type") != "fr3_eef":
        return False
    for key in NUMERIC_KEYS:
        names = info.get("features", {}).get(key, {}).get("names", [])
        if not names or not all(k in names for k in EULER_ORI_KEYS):
            return False
    return True


def _updated_feature_names(old_names: list[str]) -> list[str]:
    expected = [
        f"{n}.pos" for n in (*FR3_EEF_ARM_POSE_NAMES, *FR3_EEF_HAND_JOINT_NAMES)
    ]
    if not all(k in old_names for k in ("ee_x.pos", "ee_y.pos", "ee_z.pos", *EULER_ORI_KEYS)):
        raise ValueError(f"Unexpected fr3_eef names: {old_names}")
    if len(old_names) != 12:
        raise ValueError(f"Expected 12 feature names before migration, got {len(old_names)}")
    if len(expected) != 13:
        raise ValueError(f"Expected 13 feature names after migration, got {len(expected)}")
    return expected


def _euler_vec_to_quat_vec(vec: np.ndarray) -> np.ndarray:
    """12D (xyz + rpy + 6 hand) -> 13D (xyz + quat + 6 hand)."""
    v = np.asarray(vec, dtype=np.float64)
    if v.shape[-1] != 12:
        raise ValueError(f"Expected 12D vector, got shape {v.shape}")
    pos = v[..., :3]
    euler = v[..., 3:6]
    hand = v[..., 6:12]
    quat = Rotation.from_euler("xyz", euler).as_quat()
    return np.concatenate([pos, quat, hand], axis=-1)


def _make_global_quat_ref(state_ep0_q: np.ndarray) -> np.ndarray:
    """Dataset-wide quaternion hemisphere (same physical pose -> same coeffs)."""
    q_ref = normalize_quat_xyzw(state_ep0_q)
    if q_ref[3] < 0.0:
        q_ref = -q_ref
    return q_ref


def _apply_global_quat_ref(arr: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    out = arr.copy()
    for i in range(len(out)):
        out[i, 3:7] = align_quat_hemisphere(out[i, 3:7], q_ref)
    return out


def _align_episode_pair(action: np.ndarray, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Within-episode temporal continuity; action[0] anchored to obs[0]."""
    state_out = state.copy()
    action_out = action.copy()
    prev_obs: np.ndarray | None = None
    prev_act: np.ndarray | None = None
    for i in range(len(state_out)):
        q_raw = normalize_quat_xyzw(state_out[i, 3:7])
        q_obs = prepare_quat_xyzw(q_raw, q_prev=prev_obs)
        prev_obs = q_obs.copy()
        state_out[i, 3:7] = q_obs

        q_act_raw = normalize_quat_xyzw(action_out[i, 3:7])
        anchor = q_obs if prev_act is None else None
        q_act = prepare_quat_xyzw(q_act_raw, q_prev=prev_act, q_anchor=anchor)
        prev_act = q_act.copy()
        action_out[i, 3:7] = q_act
    return action_out, state_out


def _finalize_episode_quats(
    action: np.ndarray,
    state: np.ndarray,
    q_ref: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Same temporal rules as live record; optional global ref for legacy Euler datasets."""
    action, state = _align_episode_pair(action, state)
    if q_ref is not None:
        state = _apply_global_quat_ref(state, q_ref)
        action = _apply_global_quat_ref(action, q_ref)
        action, state = _align_episode_pair(action, state)
    return action, state


def _update_info(info: dict) -> dict:
    info = json.loads(json.dumps(info))
    for key in NUMERIC_KEYS:
        feat = info["features"][key]
        old_names = list(feat["names"])
        feat["names"] = _updated_feature_names(old_names)
        feat["shape"] = [len(feat["names"])]
    return info


def _copy_tree(src: Path, dst: Path, symlink_videos: bool) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    for name in ("meta", "data", "videos"):
        s = src / name
        if not s.exists():
            continue
        d = dst / name
        if d.exists():
            shutil.rmtree(d)
        if name == "videos" and symlink_videos:
            d.symlink_to(s.resolve())
        else:
            shutil.copytree(s, d)


def _process_data_parquet(
    src_path: Path,
    dst_path: Path,
    q_ref: np.ndarray | None,
) -> dict[int, dict]:
    """Transform one data parquet file; return per-episode stats inputs."""
    df = pd.read_parquet(src_path)
    episode_stats_inputs: dict[int, dict] = {}

    for ep in sorted(df["episode_index"].unique()):
        mask = df["episode_index"] == ep
        idx = df.index[mask]
        actions = np.stack(df.loc[mask, "action"].values).astype(np.float64)
        states = np.stack(df.loc[mask, "observation.state"].values).astype(np.float64)

        actions = np.stack([_euler_vec_to_quat_vec(a) for a in actions])
        states = np.stack([_euler_vec_to_quat_vec(s) for s in states])
        actions, states = _finalize_episode_quats(actions, states, q_ref)

        row_index = df.index[mask]
        df.loc[mask, "action"] = pd.Series(
            [actions[i].astype(np.float32) for i in range(len(actions))],
            index=row_index,
        )
        df.loc[mask, "observation.state"] = pd.Series(
            [states[i].astype(np.float32) for i in range(len(states))],
            index=row_index,
        )

        episode_stats_inputs[int(ep)] = {
            "action": actions,
            "observation.state": states,
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
    """Rewrite meta/episodes parquet with updated per-episode stats columns."""
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
    # Keep image/video stats from source global stats if present.
    src_stats_path = src_root / "meta" / "stats.json"
    if src_stats_path.exists():
        with open(src_stats_path) as f:
            src_stats = json.load(f)
        for key, value in src_stats.items():
            if key not in global_stats and key not in NUMERIC_KEYS:
                global_stats[key] = value

    write_stats(global_stats, dst_root)


def _resolve_global_quat_ref(src_root: Path, global_quat_canonical: bool) -> np.ndarray | None:
    if not global_quat_canonical:
        return None
    df = pd.read_parquet(next((src_root / "data").glob("*/*.parquet")))
    ep0 = int(sorted(df["episode_index"].unique())[0])
    s0 = np.stack(df.loc[df["episode_index"] == ep0, "observation.state"].values)[0]
    s0q = _euler_vec_to_quat_vec(s0.astype(np.float64))
    q_ref = _make_global_quat_ref(s0q[3:7])
    logger.info("Global quat ref (ep0 obs, qw>=0): %s", q_ref)
    return q_ref


def migrate_dataset(
    src_root: Path,
    dst_root: Path,
    *,
    symlink_videos: bool = True,
    overwrite: bool = False,
    global_quat_canonical: bool = True,
) -> None:
    src_root = src_root.resolve()
    dst_root = dst_root.resolve()

    if dst_root.exists():
        if not overwrite:
            raise FileExistsError(f"Output exists: {dst_root} (use --overwrite)")
        shutil.rmtree(dst_root)

    with open(src_root / "meta" / "info.json") as f:
        info = json.load(f)

    if not _is_migratable_fr3_eef(info):
        raise ValueError(
            f"{src_root} is not a migratable fr3_eef Euler dataset "
            f"(need robot_type=fr3_eef and {EULER_ORI_KEYS} in action/state)."
        )

    logger.info("Migrating %s -> %s", src_root, dst_root)
    _copy_tree(src_root, dst_root, symlink_videos=symlink_videos)

    info = _update_info(info)
    write_info(info, dst_root)

    q_ref = _resolve_global_quat_ref(src_root, global_quat_canonical)

    src_data_files = sorted((src_root / "data").glob("*/*.parquet"))
    episode_stats_by_ep: dict[int, dict] = {}

    for src_pq in tqdm(src_data_files, desc="data parquet"):
        rel = src_pq.relative_to(src_root / "data")
        dst_pq = dst_root / "data" / rel
        ep_partial = _process_data_parquet(src_pq, dst_pq, q_ref)
        for ep_idx, data in ep_partial.items():
            if ep_idx in episode_stats_by_ep:
                raise RuntimeError(f"Duplicate episode_index {ep_idx} across data files")
            episode_stats_by_ep[ep_idx] = data

    _rebuild_episodes_meta(src_root, dst_root, info, episode_stats_by_ep)
    logger.info("Done: %s (episodes=%d)", dst_root, len(episode_stats_by_ep))


def _discover_datasets(input_root: Path, pattern: str) -> list[Path]:
    out = []
    for p in sorted(input_root.iterdir()):
        if not p.is_dir():
            continue
        if not p.match(pattern):
            continue
        info_path = p / "meta" / "info.json"
        if info_path.exists():
            out.append(p)
    return out


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--input-dir", type=Path, help="Single dataset root (e.g. data/redcube1)")
    src.add_argument(
        "--input-root",
        type=Path,
        help="Parent directory; migrate children matching --pattern",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output dataset root (with --input-dir)",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Parent for outputs (with --input-root); writes output-root/<name>",
    )
    parser.add_argument(
        "--pattern",
        type=str,
        default="*",
        help="Glob on child names under --input-root (default: *)",
    )
    parser.add_argument(
        "--symlink-videos",
        action="store_true",
        default=True,
        help="Symlink videos/ instead of copying (default: true)",
    )
    parser.add_argument(
        "--copy-videos",
        action="store_true",
        help="Copy videos/ instead of symlinking",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace output directory if it exists",
    )
    parser.add_argument(
        "--global-quat-canonical",
        action="store_true",
        default=True,
        help="Sign-align all quats to dataset q_ref (default; required for offline Euler)",
    )
    parser.add_argument(
        "--no-global-quat-canonical",
        action="store_false",
        dest="global_quat_canonical",
        help="Per-episode align only (debug; not for production Euler migration)",
    )
    args = parser.parse_args()
    symlink_videos = not args.copy_videos
    global_quat_canonical = args.global_quat_canonical

    if args.input_dir is not None:
        if args.output_dir is None:
            parser.error("--output-dir is required with --input-dir")
        migrate_dataset(
            args.input_dir,
            args.output_dir,
            symlink_videos=symlink_videos,
            overwrite=args.overwrite,
            global_quat_canonical=global_quat_canonical,
        )
        return 0

    if args.output_root is None:
        parser.error("--output-root is required with --input-root")

    datasets = _discover_datasets(args.input_root.resolve(), args.pattern)
    if not datasets:
        logger.error("No datasets found under %s matching %s", args.input_root, args.pattern)
        return 1

    for src in datasets:
        dst = args.output_root.resolve() / src.name
        migrate_dataset(
            src,
            dst,
            symlink_videos=symlink_videos,
            overwrite=args.overwrite,
            global_quat_canonical=global_quat_canonical,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
