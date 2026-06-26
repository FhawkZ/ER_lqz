"""Serialization helpers for dataset writers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def jsonable(value: Any, asset_dir: Path | None = None, stem: str = "asset") -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (list, tuple)):
        return [jsonable(v, asset_dir, stem) for v in value]
    if isinstance(value, dict):
        return {str(k): jsonable(v, asset_dir, f"{stem}_{k}") for k, v in value.items()}
    if hasattr(value, "tolist"):
        try:
            return value.tolist()
        except Exception:
            pass
    if hasattr(value, "shape") and hasattr(value, "dtype") and asset_dir is not None:
        try:
            import numpy as np

            asset_dir.mkdir(parents=True, exist_ok=True)
            path = asset_dir / f"{stem}.npy"
            np.save(path, value)
            return {"array_path": str(path), "shape": list(value.shape), "dtype": str(value.dtype)}
        except Exception:
            pass
    try:
        json.dumps(value)
        return value
    except TypeError:
        return repr(value)
