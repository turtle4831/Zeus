"""TCP client used by the laptop driver station."""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable

from RobotSide.Networking.protocol import (
    DEFAULT_PORT,
    ControlMessage,
    HeartbeatMessage,
    LineBuffer,
    TelemetryMessage,
    encode_message,
)


class DriverStationClient:
    def __init__(
        self,
        host: str = "192.168.4.1",
        port: int = DEFAULT_PORT,
        control_rate_hz: float = 50.0,
        heartbeat_interval_s: float = 1.0,
    ):
        self.host = host
        self.port = port
        self.control_interval_s = 1.0 / control_rate_hz
        self.heartbeat_interval_s = heartbeat_interval_s

        self._socket: socket.socket | None = None
        self._running = False
        self._send_thread: threading.Thread | None = None
        self._recv_thread: threading.Thread | None = None
        self._control_provider: Callable[[], ControlMessage] | None = None
        self._on_telemetry: Callable[[TelemetryMessage], None] | None = None
        self._on_connection_changed: Callable[[bool], None] | None = None
        self._last_heartbeat = 0.0

    def connect(
        self,
        control_provider: Callable[[], ControlMessage],
        on_telemetry: Callable[[TelemetryMessage], None] | None = None,
        on_connection_changed: Callable[[bool], None] | None = None,
    ) -> None:
        if self._running:
            return
        self._control_provider = control_provider
        self._on_telemetry = on_telemetry
        self._on_connection_changed = on_connection_changed
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(2.0)
        self._socket.connect((self.host, self.port))
        self._socket.settimeout(0.5)
        self._running = True
        if self._on_connection_changed:
            self._on_connection_changed(True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread.start()
        self._recv_thread.start()

    def disconnect(self) -> None:
        self._running = False
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        if self._on_connection_changed:
            self._on_connection_changed(False)

    def is_connected(self) -> bool:
        return self._running and self._socket is not None

    def _send_loop(self) -> None:
        while self._running and self._socket:
            if self._control_provider:
                try:
                    self._socket.sendall(encode_message(self._control_provider()))
                except OSError:
                    self.disconnect()
                    break
            now = time.time()
            if now - self._last_heartbeat >= self.heartbeat_interval_s:
                try:
                    self._socket.sendall(encode_message(HeartbeatMessage(role="driver_station")))
                    self._last_heartbeat = now
                except OSError:
                    self.disconnect()
                    break
            time.sleep(self.control_interval_s)

    def _recv_loop(self) -> None:
        buffer = LineBuffer()
        while self._running and self._socket:
            try:
                data = self._socket.recv(4096)
            except TimeoutError:
                continue
            except OSError:
                self.disconnect()
                break
            if not data:
                self.disconnect()
                break
            for message in buffer.feed(data):
                if isinstance(message, TelemetryMessage) and self._on_telemetry:
                    self._on_telemetry(message)
