"""MCP SDK（``mocap_robotapi`` + ``lib/*.so``）。"""

from __future__ import annotations

import sys
from pathlib import Path

_SDK_DIR = Path(__file__).resolve().parent
_dir = str(_SDK_DIR)
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from mocap_robotapi import (  # noqa: E402
    MCPApplication,
    MCPAvatar,
    MCPEventType,
    MCPSettings,
)

__all__ = [
    "MCPApplication",
    "MCPAvatar",
    "MCPEventType",
    "MCPSettings",
]
