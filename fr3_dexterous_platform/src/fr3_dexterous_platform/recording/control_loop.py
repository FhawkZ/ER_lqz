"""Rigorous teleoperation/policy data collection loop."""

from __future__ import annotations

from fr3_dexterous_platform.interfaces import ActionSource, EpisodeWriter, RobotBackend
from fr3_dexterous_platform.recording.alignment import check_freshness
from fr3_dexterous_platform.schemas import CollectionConfig, StepRecord
from fr3_dexterous_platform.utils.time import now_s, sleep_until_next


class CollectionResult:
    def __init__(self, written_frames: int, dropped_frames: int, issues: list[str]):
        self.written_frames = written_frames
        self.dropped_frames = dropped_frames
        self.issues = issues


def collect_episode(
    robot: RobotBackend,
    action_source: ActionSource,
    writer: EpisodeWriter,
    config: CollectionConfig,
) -> CollectionResult:
    """Run one episode: observe, compute action, send action, write step."""

    if config.frames is None and config.duration_s is None:
        raise ValueError("CollectionConfig requires either frames or duration_s")

    robot.connect()
    action_source.connect()
    action_source.reset()
    writer.start_episode(config)

    written = 0
    dropped = 0
    issues: list[str] = []
    period_s = 1.0 / float(config.fps)
    start = now_s()
    frame_index = 0

    try:
        while True:
            if config.frames is not None and frame_index >= config.frames:
                break
            if config.duration_s is not None and now_s() - start >= config.duration_s:
                break

            loop_start = now_s()
            observation = robot.get_observation()
            raw_action = action_source.get_action(observation)

            freshness_issues = []
            if config.freshness is not None:
                freshness_issues = check_freshness(observation, raw_action.source_time, config.freshness)

            sent_action = robot.send_action(raw_action)
            loop_end = now_s()

            diagnostics = {
                "loop_ms": (loop_end - loop_start) * 1000.0,
                "freshness_issues": [issue.__dict__ for issue in freshness_issues],
            }
            step = StepRecord(
                episode_index=config.episode_index,
                frame_index=frame_index,
                task=config.task,
                loop_start_time=loop_start,
                observation=observation,
                raw_action=raw_action,
                sent_action=sent_action,
                loop_end_time=loop_end,
                diagnostics=diagnostics,
            )

            if freshness_issues and config.drop_stale_frames:
                dropped += 1
                issues.extend(f"{issue.key}: {issue.message}" for issue in freshness_issues)
            else:
                writer.write_step(step)
                written += 1

            frame_index += 1
            sleep_until_next(loop_start, period_s)
    finally:
        writer.close_episode()
        writer.finalize()
        action_source.disconnect()
        robot.disconnect()

    return CollectionResult(written_frames=written, dropped_frames=dropped, issues=issues)
