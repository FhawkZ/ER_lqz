"""HTTP policy server/client with standard-library dependencies only."""

from __future__ import annotations

import json
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from fr3_dexterous_platform.inference.runtime import EchoPolicy, PolicyRuntime
from fr3_dexterous_platform.schemas import Action, Observation, StampedValue
from fr3_dexterous_platform.utils.time import now_s


def observation_to_payload(observation: Observation) -> dict[str, Any]:
    return {
        "receive_time": observation.receive_time,
        "values": {
            key: {
                "value": stamped.value,
                "source_time": stamped.source_time,
                "receive_time": stamped.receive_time,
                "topic": stamped.topic,
                "sequence_id": stamped.sequence_id,
            }
            for key, stamped in observation.values.items()
        },
    }


def observation_from_payload(payload: dict[str, Any]) -> Observation:
    values = {
        key: StampedValue(
            value=raw["value"],
            source_time=float(raw["source_time"]),
            receive_time=float(raw["receive_time"]),
            topic=raw.get("topic", ""),
            sequence_id=raw.get("sequence_id"),
        )
        for key, raw in payload["values"].items()
    }
    return Observation(values=values, receive_time=float(payload["receive_time"]))


class RemotePolicyClient:
    def __init__(self, server_address: str, timeout_s: float = 5.0):
        self.server_address = server_address.rstrip("/")
        self.timeout_s = timeout_s

    def predict(self, observation: Observation) -> Action:
        request_t = now_s()
        body = json.dumps({"observation": observation_to_payload(observation)}).encode("utf-8")
        request = urllib.request.Request(
            self.server_address + "/predict",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
        receive_t = now_s()
        action = payload["action"]
        return Action(
            values=action["values"],
            source_time=float(action["source_time"]),
            receive_time=receive_t,
            trace={
                **action.get("trace", {}),
                "network_request_time": request_t,
                "network_receive_time": receive_t,
                "roundtrip_ms": (receive_t - request_t) * 1000.0,
            },
        )


class PolicyHttpServer:
    def __init__(self, host: str, port: int, policy: PolicyRuntime | None = None):
        self.host = host
        self.port = port
        self.policy = policy or EchoPolicy()
        self.httpd: ThreadingHTTPServer | None = None
        self.thread: threading.Thread | None = None

    def _make_httpd(self) -> ThreadingHTTPServer:
        policy = self.policy

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):  # noqa: N802
                if self.path != "/predict":
                    self.send_response(404)
                    self.end_headers()
                    return
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                observation = observation_from_payload(payload["observation"])
                action = policy.predict(observation)
                response = json.dumps(
                    {
                        "action": {
                            "values": dict(action.values),
                            "source_time": action.source_time,
                            "receive_time": action.receive_time,
                            "trace": dict(action.trace),
                        }
                    },
                    ensure_ascii=False,
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(response)))
                self.end_headers()
                self.wfile.write(response)

            def log_message(self, format, *args):  # noqa: A002
                return

        return ThreadingHTTPServer((self.host, self.port), Handler)

    def start(self) -> None:
        self.httpd = self._make_httpd()
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def serve_forever(self) -> None:
        if self.httpd is None:
            self.httpd = self._make_httpd()
        assert self.httpd is not None
        self.httpd.serve_forever()

    def stop(self) -> None:
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
