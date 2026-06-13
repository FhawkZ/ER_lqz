"""Multi-format dataset writers."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Iterable

from fr3_dexterous_platform.interfaces import EpisodeWriter
from fr3_dexterous_platform.schemas import CollectionConfig, StepRecord
from fr3_dexterous_platform.utils.serialize import jsonable


def step_to_dict(step: StepRecord, asset_dir: Path | None = None) -> dict:
    obs = {
        key: {
            "value": jsonable(stamped.value, asset_dir, f"ep{step.episode_index:06d}_f{step.frame_index:06d}_{key}"),
            "source_time": stamped.source_time,
            "receive_time": stamped.receive_time,
            "publish_time": stamped.publish_time,
            "sequence_id": stamped.sequence_id,
            "topic": stamped.topic,
        }
        for key, stamped in step.observation.values.items()
    }
    return {
        "episode_index": step.episode_index,
        "frame_index": step.frame_index,
        "timestamp": step.timestamp,
        "task": step.task,
        "loop_start_time": step.loop_start_time,
        "loop_end_time": step.loop_end_time,
        "observation": obs,
        "raw_action": {
            "values": jsonable(dict(step.raw_action.values), asset_dir, f"ep{step.episode_index:06d}_f{step.frame_index:06d}_raw_action"),
            "source_time": step.raw_action.source_time,
            "receive_time": step.raw_action.receive_time,
            "trace": jsonable(dict(step.raw_action.trace), asset_dir, "trace"),
        },
        "sent_action": {
            "values": jsonable(dict(step.sent_action.values), asset_dir, f"ep{step.episode_index:06d}_f{step.frame_index:06d}_sent_action"),
            "source_time": step.sent_action.source_time,
            "receive_time": step.sent_action.receive_time,
            "trace": jsonable(dict(step.sent_action.trace), asset_dir, "sent_trace"),
        },
        "diagnostics": jsonable(dict(step.diagnostics), asset_dir, "diagnostics"),
    }


class JsonlWriter(EpisodeWriter):
    def __init__(self, root: Path, name: str = "jsonl"):
        self.root = root / name
        self.handle = None
        self.asset_dir: Path | None = None

    def start_episode(self, config: CollectionConfig) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.asset_dir = self.root / "assets"
        path = self.root / f"episode_{config.episode_index:06d}.jsonl"
        self.handle = path.open("w", encoding="utf-8")
        (self.root / "meta.json").write_text(json.dumps(asdict(config), indent=2, ensure_ascii=False), encoding="utf-8")

    def write_step(self, step: StepRecord) -> None:
        assert self.handle is not None
        self.handle.write(json.dumps(step_to_dict(step, self.asset_dir), ensure_ascii=False) + "\n")

    def close_episode(self) -> None:
        if self.handle:
            self.handle.close()
            self.handle = None

    def finalize(self) -> None:
        self.close_episode()


class StagingFormatWriter(JsonlWriter):
    """JSONL staging writer for training-framework-specific converters."""

    def __init__(self, root: Path, name: str, schema_name: str):
        super().__init__(root, name)
        self.schema_name = schema_name

    def start_episode(self, config: CollectionConfig) -> None:
        super().start_episode(config)
        info = {
            "schema": self.schema_name,
            "task": config.task,
            "fps": config.fps,
            "notes": "Staging format. Convert with framework-specific tools if strict binary layout is required.",
        }
        (self.root / "dataset_info.json").write_text(json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")


class Hdf5Writer(EpisodeWriter):
    def __init__(self, root: Path):
        self.root = root / "hdf5"
        self.frames: list[str] = []
        self.config: CollectionConfig | None = None

    def start_episode(self, config: CollectionConfig) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.frames = []
        self.config = config

    def write_step(self, step: StepRecord) -> None:
        self.frames.append(json.dumps(step_to_dict(step, self.root / "assets"), ensure_ascii=False))

    def close_episode(self) -> None:
        if self.config is None:
            return
        try:
            import h5py
        except ImportError as exc:
            raise RuntimeError("hdf5 output requires optional dependency h5py") from exc
        path = self.root / f"episode_{self.config.episode_index:06d}.hdf5"
        with h5py.File(path, "w") as h5:
            dt = h5py.string_dtype(encoding="utf-8")
            h5.create_dataset("frames_json", data=self.frames, dtype=dt)
            h5.attrs["task"] = self.config.task
            h5.attrs["fps"] = self.config.fps
        self.config = None

    def finalize(self) -> None:
        self.close_episode()


class MultiFormatWriter(EpisodeWriter):
    def __init__(self, writers: Iterable[EpisodeWriter]):
        self.writers = list(writers)

    @classmethod
    def from_formats(cls, root: Path, formats: Iterable[str]) -> "MultiFormatWriter":
        writers: list[EpisodeWriter] = []
        for fmt in formats:
            normalized = fmt.strip().lower()
            if not normalized:
                continue
            if normalized == "jsonl":
                writers.append(JsonlWriter(root))
            elif normalized == "lerobot":
                writers.append(StagingFormatWriter(root, "lerobot", "lerobot_staging_v1"))
            elif normalized == "droid":
                writers.append(StagingFormatWriter(root, "droid", "droid_staging_v1"))
            elif normalized == "openpi":
                writers.append(StagingFormatWriter(root, "openpi", "openpi_staging_v1"))
            elif normalized in {"h5", "hdf5", "robomimic"}:
                writers.append(Hdf5Writer(root))
            else:
                raise ValueError(f"Unsupported output format: {fmt}")
        if not writers:
            raise ValueError("At least one output format is required")
        return cls(writers)

    def start_episode(self, config: CollectionConfig) -> None:
        for writer in self.writers:
            writer.start_episode(config)

    def write_step(self, step: StepRecord) -> None:
        for writer in self.writers:
            writer.write_step(step)

    def close_episode(self) -> None:
        for writer in self.writers:
            writer.close_episode()

    def finalize(self) -> None:
        for writer in self.writers:
            writer.finalize()
