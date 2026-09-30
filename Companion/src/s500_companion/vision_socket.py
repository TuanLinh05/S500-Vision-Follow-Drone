from __future__ import annotations

import ipaddress
import socket

from .models import GuidanceProposal, VisionStatus
from .vision_protocol import VisionProtocolError, parse_vision_packet


class LocalVisionReceiver:
    """Non-blocking loopback-only receiver for detector/tracker packets."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5800) -> None:
        address = ipaddress.ip_address(host)
        if not address.is_loopback:
            raise ValueError("Vision UDP must bind to a loopback address")
        self.host = host
        self.port = port
        self._socket: socket.socket | None = None

    def open(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.bind((self.host, self.port))
        self.port = int(self._socket.getsockname()[1])

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
        self._socket = None

    def __enter__(self) -> "LocalVisionReceiver":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def receive_latest(self) -> tuple[VisionStatus, GuidanceProposal] | None:
        if self._socket is None:
            raise RuntimeError("Vision receiver is not open")
        latest: bytes | None = None
        while True:
            try:
                payload, source = self._socket.recvfrom(65535)
            except BlockingIOError:
                break
            if not ipaddress.ip_address(source[0]).is_loopback:
                continue
            latest = payload
        if latest is None:
            return None
        try:
            return parse_vision_packet(latest)
        except VisionProtocolError:
            return None

