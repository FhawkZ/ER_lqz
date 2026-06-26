"""Freshness and alignment checks."""

from __future__ import annotations

from fr3_dexterous_platform.schemas import FreshnessIssue, FreshnessSpec, Observation


def check_freshness(observation: Observation, target_time: float, spec: FreshnessSpec) -> list[FreshnessIssue]:
    issues: list[FreshnessIssue] = []
    for key in spec.required_keys:
        if key not in observation.values:
            issues.append(FreshnessIssue(key=key, message="missing required observation key"))
    for key, max_age_s in spec.max_age_s.items():
        stamped = observation.values.get(key)
        if stamped is None:
            continue
        age_s = stamped.age_at(target_time)
        if age_s < -1e-6:
            issues.append(FreshnessIssue(key=key, message="observation comes from the future", age_s=age_s))
        elif age_s > max_age_s:
            issues.append(
                FreshnessIssue(
                    key=key,
                    message=f"observation age {age_s * 1000:.3f}ms exceeds {max_age_s * 1000:.3f}ms",
                    age_s=age_s,
                )
            )
    return issues
