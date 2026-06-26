"""Quaternion helpers using xyzw ordering."""

from __future__ import annotations

import math


def normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)  # type: ignore[return-value]


def dot(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def align_to_previous(
    q: tuple[float, float, float, float],
    previous: tuple[float, float, float, float] | None,
) -> tuple[float, float, float, float]:
    qn = normalize(q)
    if previous is not None and dot(qn, previous) < 0.0:
        return tuple(-v for v in qn)  # type: ignore[return-value]
    return qn


def conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def to_rotvec(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = normalize(q)
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8:
        return (0.0, 0.0, 0.0)
    return (x / s * angle, y / s * angle, z / s * angle)


def from_rotvec(rv: tuple[float, float, float]) -> tuple[float, float, float, float]:
    rx, ry, rz = rv
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return normalize((rx * s, ry * s, rz * s, math.cos(angle / 2.0)))
