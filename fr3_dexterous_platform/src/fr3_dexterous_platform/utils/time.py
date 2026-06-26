"""Timing helpers."""

from __future__ import annotations

import time


def now_s() -> float:
    return time.perf_counter()


def sleep_until_next(loop_start: float, period_s: float) -> None:
    remain = period_s - (time.perf_counter() - loop_start)
    if remain > 0:
        time.sleep(remain)
