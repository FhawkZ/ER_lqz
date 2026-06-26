# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

"""Record policy-server inference inputs (images, state) and output action chunks."""

from __future__ import annotations

import csv
import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import torch

logger = logging.getLogger(__name__)

_DEFAULT_ACTION_NAMES = [
    "ee_x",
    "ee_y",
    "ee_z",
    "ori_qx",
    "ori_qy",
    "ori_qz",
    "ori_qw",
    "hand_0",
    "hand_1",
    "hand_2",
    "hand_3",
    "hand_4",
    "hand_5",
]


def _tensor_to_uint8_image(tensor: torch.Tensor) -> torch.Tensor:
    """Convert (C, H, W) or (1, C, H, W) tensor to uint8 HWC for saving."""
    img = tensor.detach().cpu()
    while img.ndim > 3:
        img = img.squeeze(0)
    if img.dtype != torch.uint8:
        if img.max() <= 1.0:
            img = (img.clamp(0, 1) * 255.0).to(torch.uint8)
        else:
            img = img.clamp(0, 255).to(torch.uint8)
    return img.permute(1, 2, 0).contiguous()


class InferenceTraceRecorder:
    """Writes inference traces under one session directory per server start."""

    def __init__(self, trace_dir: str | Path, enabled: bool = True) -> None:
        self.enabled = enabled and bool(str(trace_dir).strip())
        self.trace_dir = Path(trace_dir) if self.enabled else None
        self.session_dir: Path | None = None
        if self.enabled:
            self.trace_dir.mkdir(parents=True, exist_ok=True)
            session_id = f"session_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}"
            self.session_dir = self.trace_dir / session_id
            self.session_dir.mkdir(parents=True, exist_ok=True)
            session_meta = {
                "session_id": session_id,
                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                "trace_dir": str(self.trace_dir),
            }
            (self.session_dir / "session.json").write_text(
                json.dumps(session_meta, indent=2), encoding="utf-8"
            )
            logger.info("Inference trace session: %s", self.session_dir)

    def record(
        self,
        *,
        timestep: int,
        timestamp: float,
        policy_type: str,
        pretrained_path: str,
        raw_observation: dict[str, Any],
        observation: dict[str, Any],
        action_chunk: torch.Tensor,
        action_names: list[str] | None = None,
    ) -> Path | None:
        if not self.enabled or self.session_dir is None:
            return None

        obs_dir = self.session_dir / f"obs_{timestep:06d}"
        obs_dir.mkdir(parents=True, exist_ok=True)

        meta = {
            "timestep": timestep,
            "timestamp": timestamp,
            "policy_type": policy_type,
            "pretrained_path": pretrained_path,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        (obs_dir / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

        if "task" in raw_observation:
            (obs_dir / "task.txt").write_text(str(raw_observation["task"]), encoding="utf-8")

        state = observation.get("observation.state")
        if state is not None:
            state_vec = state.detach().cpu().flatten().tolist() if isinstance(state, torch.Tensor) else state
            (obs_dir / "state.json").write_text(json.dumps({"observation.state": state_vec}), encoding="utf-8")

        images_dir = obs_dir / "images"
        images_dir.mkdir(exist_ok=True)
        for key, value in observation.items():
            if "image" not in key or not isinstance(value, torch.Tensor):
                continue
            try:
                from PIL import Image

                img = _tensor_to_uint8_image(value)
                safe_name = key.replace("/", "_").replace(".", "_")
                Image.fromarray(img.numpy()).save(images_dir / f"{safe_name}.png")
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to save image %s: %s", key, exc)

        names = action_names or _DEFAULT_ACTION_NAMES
        chunk = action_chunk.detach().cpu()
        if chunk.ndim == 3:
            chunk = chunk.squeeze(0)
        rows: list[dict[str, float | int]] = []
        for step_i in range(chunk.shape[0]):
            row: dict[str, float | int] = {"step": step_i}
            for dim_i, name in enumerate(names[: chunk.shape[1]]):
                row[name] = float(chunk[step_i, dim_i].item())
            rows.append(row)

        csv_path = obs_dir / "actions.csv"
        if rows:
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)

        logger.info("Recorded inference trace: %s", obs_dir)
        return obs_dir

    def record_error(
        self,
        *,
        stage: str,
        error: str,
        policy_type: str | None = None,
        pretrained_path: str | None = None,
        timestep: int | None = None,
        extra: dict[str, Any] | None = None,
    ) -> Path | None:
        """Save inference / policy-load failures for offline debugging."""
        if not self.enabled or self.session_dir is None:
            return None

        err_dir = self.session_dir / "errors" / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        err_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "stage": stage,
            "error": error,
            "policy_type": policy_type,
            "pretrained_path": pretrained_path,
            "timestep": timestep,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            **(extra or {}),
        }
        (err_dir / "error.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        logger.error("Recorded inference error trace: %s | %s", err_dir, error)
        return err_dir
