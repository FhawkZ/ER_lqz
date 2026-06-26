"""Isaac UDP/JSON action source."""

from __future__ import annotations

import json
import socket
import threading
import time

from fr3_dexterous_platform.interfaces import ActionSource
from fr3_dexterous_platform.isaac.protocol import IsaacPacket, parse_isaac_packet
from fr3_dexterous_platform.schemas import Action, Observation
from fr3_dexterous_platform.teleop.mocap_delta import DeltaMocapRetargeter
from fr3_dexterous_platform.utils.time import now_s


class IsaacUdpActionSource(ActionSource):
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 15050,
        timeout_s: float = 5.0,
        buffer_size: int = 65535,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.buffer_size = buffer_size
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_packet: IsaacPacket | None = None
        self._retargeter = DeltaMocapRetargeter()

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()
        self._sock = None

    def reset(self) -> None:
        self._retargeter.reset()

    def get_action(self, observation: Observation) -> Action:
        packet = self._wait_for_packet()
        receive = now_s()
        if packet.action is not None:
            return Action(
                values=dict(packet.action),
                source_time=packet.source_time,
                receive_time=receive,
                trace={"source": "isaac_udp_action", "sequence_id": packet.sequence_id},
            )
        if packet.mocap_pose is None:
            raise RuntimeError("Isaac packet has neither action nor mocap pose")
        values = self._retargeter.action_values(observation, packet.mocap_pose, packet.hand_joints)
        return Action(
            values=values,
            source_time=packet.source_time,
            receive_time=receive,
            trace={"source": "isaac_udp_mocap", "sequence_id": packet.sequence_id},
        )

    def _recv_loop(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(self.buffer_size)
            except socket.timeout:
                continue
            payload = json.loads(data.decode("utf-8"))
            packet = parse_isaac_packet(payload)
            with self._lock:
                self._latest_packet = packet

    def _wait_for_packet(self) -> IsaacPacket:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_packet is not None:
                    return self._latest_packet
            time.sleep(0.005)
        raise TimeoutError(f"Timeout waiting for Isaac UDP packet on {self.host}:{self.port}")
