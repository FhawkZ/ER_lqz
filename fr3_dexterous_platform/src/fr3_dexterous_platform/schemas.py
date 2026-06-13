"""Core data schemas for collection, alignment, and export."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class StampedValue:
    value: Any
    source_time: float
    receive_time: float
    topic: str = ""
    publish_time: float | None = None
    sequence_id: int | None = None

    def age_at(self, timestamp: float) -> float:
        return timestamp - self.source_time


@dataclass(frozen=True)
class Observation:
    values: Mapping[str, StampedValue]
    receive_time: float

    def plain(self) -> dict[str, Any]:
        return {key: stamped.value for key, stamped in self.values.items()}


@dataclass(frozen=True)
class Action:
    values: Mapping[str, Any]
    source_time: float
    receive_time: float
    trace: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StepRecord:
    episode_index: int
    frame_index: int
    task: str
    loop_start_time: float
    observation: Observation
    raw_action: Action
    sent_action: Action
    loop_end_time: float
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    @property
    def timestamp(self) -> float:
        return self.raw_action.source_time


@dataclass(frozen=True)
class FreshnessSpec:
    max_age_s: Mapping[str, float]
    required_keys: tuple[str, ...] = ()


@dataclass(frozen=True)
class FreshnessIssue:
    key: str
    message: str
    age_s: float | None = None


@dataclass(frozen=True)
class CollectionConfig:
    task: str
    fps: float = 30.0
    frames: int | None = None
    duration_s: float | None = None
    episode_index: int = 0
    freshness: FreshnessSpec | None = None
    drop_stale_frames: bool = True
