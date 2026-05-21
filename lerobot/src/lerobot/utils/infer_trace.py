#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CSV trace logger for policy inference on real robots (fr3_eef)."""

from __future__ import annotations

import csv
import json
import logging
import os
import time
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.utils.quat import orientation_delta_deg

logger = logging.getLogger(__name__)

# Matches fr3_eef / redcube dataset feature order (7D EE quaternion).
STATE_KEYS = [
    "ee_x.pos",
    "ee_y.pos",
    "ee_z.pos",
    "ori_qx.pos",
    "ori_qy.pos",
    "ori_qz.pos",
    "ori_qw.pos",
    "hand_0.pos",
    "hand_1.pos",
    "hand_2.pos",
    "hand_3.pos",
    "hand_4.pos",
    "hand_5.pos",
]

CSV_COLUMNS = (
    ["t_wall", "t_episode", "loop_hz"]
    + [f"state_{k.replace('.pos', '')}" for k in STATE_KEYS]
    + [f"policy_{k.replace('.pos', '')}" for k in STATE_KEYS]
    + [f"sent_{k.replace('.pos', '')}" for k in STATE_KEYS]
    + ["ori_delta_deg", "policy_ori_qw", "state_ori_qw"]
)


def _vec_from_action(action: dict[str, Any], keys: list[str] = STATE_KEYS) -> list[float]:
    out: list[float] = []
    for k in keys:
        if k not in action:
            raise KeyError(f"Missing key {k!r} in action dict")
        out.append(float(action[k]))
    return out


class InferTraceLogger:
    """Append-only CSV logger; enabled when ``INFER_TRACE_DIR`` is set."""

    def __init__(self, root: Path, episode_index: int = 0) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        self.episode_index = episode_index
        self._path = self.root / f"episode_{episode_index:03d}.csv"
        self._meta_path = self.root / "meta.json"
        self._file = self._path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()
        self._start_episode_t = time.perf_counter()
        self._last_t: float | None = None
        self._row_count = 0
        logger.info("Infer trace logging to %s", self._path)

    @classmethod
    def from_env(cls, episode_index: int = 0) -> InferTraceLogger | None:
        trace_dir = os.environ.get("INFER_TRACE_DIR", "").strip()
        if not trace_dir:
            return None
        return cls(Path(trace_dir), episode_index=episode_index)

    def write_meta(self, extra: dict[str, Any]) -> None:
        meta = {"episode_index": self.episode_index, "columns": list(CSV_COLUMNS), **extra}
        if self._meta_path.exists():
            existing = json.loads(self._meta_path.read_text())
            if isinstance(existing, list):
                existing.append(meta)
                meta = existing
            else:
                meta = [existing, meta]
        self._meta_path.write_text(json.dumps(meta, indent=2))

    def log(
        self,
        *,
        obs: dict[str, Any],
        policy_action: dict[str, Any],
        sent_action: dict[str, Any],
    ) -> None:
        t_wall = time.perf_counter()
        t_episode = t_wall - self._start_episode_t
        loop_hz = 0.0 if self._last_t is None else 1.0 / max(t_wall - self._last_t, 1e-9)
        self._last_t = t_wall

        state = _vec_from_action(obs)
        policy = _vec_from_action(policy_action)
        sent = _vec_from_action(sent_action)

        state_quat = np.array(state[3:7])
        policy_quat = np.array(policy[3:7])
        ori_delta = orientation_delta_deg(state_quat, policy_quat)

        row = (
            [t_wall, t_episode, loop_hz]
            + state
            + policy
            + sent
            + [ori_delta, policy[6], state[6]]
        )
        self._writer.writerow(row)
        self._row_count += 1
        if self._row_count % 30 == 0:
            self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()
        logger.info("Infer trace saved %s (%d rows)", self._path, self._row_count)


def summarize_trace_csv(csv_path: Path) -> dict[str, Any]:
    """Quick stats for orientation debugging."""
    import pandas as pd

    df = pd.read_csv(csv_path)
    out: dict[str, Any] = {"rows": len(df), "path": str(csv_path)}
    if "loop_hz" in df.columns:
        out["loop_hz_median"] = float(df["loop_hz"].median())
        out["loop_hz_min"] = float(df["loop_hz"].min())
    if "ori_delta_deg" in df.columns:
        out["ori_delta_deg_mean"] = float(df["ori_delta_deg"].mean())
        out["ori_delta_deg_max"] = float(df["ori_delta_deg"].max())
        out["ori_delta_deg_p95"] = float(df["ori_delta_deg"].quantile(0.95))
    for col in ("state_ori_qw", "policy_ori_qw", "sent_ori_qw"):
        if col in df.columns:
            out[f"{col}_mean"] = float(df[col].mean())
            out[f"{col}_min"] = float(df[col].min())
            out[f"{col}_max"] = float(df[col].max())
    return out
