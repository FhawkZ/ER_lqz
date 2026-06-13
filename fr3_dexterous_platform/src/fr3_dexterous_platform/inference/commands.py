"""Command planners for external policy runtimes."""

from __future__ import annotations


def openpi_server_command(host: str, port: int, model_path: str) -> list[str]:
    return [
        "python",
        "-m",
        "openpi.serving.server",
        "--host",
        host,
        "--port",
        str(port),
        "--model",
        model_path,
    ]
