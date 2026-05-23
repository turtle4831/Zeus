"""TCP server on the robot that receives driver station controls and sends telemetry."""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable

from RobotSide.Networking.protocol import (
    CONTROL_TIMEOUT_S,
    DEFAULT_PORT,
    ControlMessage,
    HeartbeatMessage,
    LineBuffer,
    TelemetryMessage,
    encode_message,
)


class DriverStationServer:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        control_timeout_s: float = CONTROL_TIMEOUT_S,
        telemetry_rate_hz: float = 10.0,
    ):
        self.host = host
        self.port = port
        self.control_timeout_s = control_timeout_s
        self.telemetry_interval_s = 1.0 / telemetry_rate_hz

        self._latest_control = ControlMessage()
        self._last_control_time = 0.0
        self._lock = threading.Lock()
        self._client_socket: socket.socket | None = None
        self._client_address: tuple[str, int] | None = None
        self._running = False
        self._server_socket: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._telemetry_thread: threading.Thread | None = None
        self._telemetry_provider: Callable[[], TelemetryMessage] | None = None

    def start(self, telemetry_provider: Callable[[], TelemetryMessage] | None = None) -> None:
        if self._running:
            return
        self._telemetry_provider = telemetry_provider
        self._running = True
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self.port = self._server_socket.getsockname()[1]
        self._server_socket.listen(1)
        self._server_socket.settimeout(1.0)
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._client_socket:
            try:
                self._client_socket.close()
            except OSError:
                pass
            self._client_socket = None
        if self._server_socket:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None

    def is_connected(self) -> bool:
        with self._lock:
            return self._client_socket is not None

    def get_client_address(self) -> tuple[str, int] | None:
        with self._lock:
            return self._client_address

    def get_controls(self) -> ControlMessage:
        with self._lock:
            if time.time() - self._last_control_time > self.control_timeout_s:
                return self._latest_control.zeroed()
            return ControlMessage(
                enabled=self._latest_control.enabled,
                lx=self._latest_control.lx,
                ly=self._latest_control.ly,
                rx=self._latest_control.rx,
                ry=self._latest_control.ry,
                buttons=dict(self._latest_control.buttons),
                timestamp=self._latest_control.timestamp,
            )

    def send_telemetry(self, telemetry: TelemetryMessage) -> None:
        telemetry.connected = self.is_connected()
        self._send(telemetry)

    def _accept_loop(self) -> None:
        while self._running and self._server_socket:
            try:
                client, address = self._server_socket.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            self._handle_client(client, address)

    def _handle_client(self, client: socket.socket, address: tuple[str, int]) -> None:
        with self._lock:
            if self._client_socket:
                try:
                    self._client_socket.close()
                except OSError:
                    pass
            self._client_socket = client
            self._client_address = address

        client.settimeout(0.5)
        buffer = LineBuffer()
        try:
            while self._running:
                try:
                    data = client.recv(4096)
                except TimeoutError:
                    continue
                if not data:
                    break
                for message in buffer.feed(data):
                    if isinstance(message, ControlMessage):
                        self._store_control(message)
                    elif isinstance(message, HeartbeatMessage):
                        pass
        except OSError:
            pass
        finally:
            with self._lock:
                if self._client_socket is client:
                    self._client_socket = None
                    self._client_address = None
            try:
                client.close()
            except OSError:
                pass

    def _store_control(self, control: ControlMessage) -> None:
        with self._lock:
            self._latest_control = control
            self._last_control_time = time.time()

    def _telemetry_loop(self) -> None:
        while self._running:
            if self._telemetry_provider and self.is_connected():
                try:
                    telemetry = self._telemetry_provider()
                    self.send_telemetry(telemetry)
                except Exception:
                    pass
            time.sleep(self.telemetry_interval_s)

    def _send(self, message: TelemetryMessage | HeartbeatMessage) -> None:
        with self._lock:
            client = self._client_socket
        if not client:
            return
        try:
            client.sendall(encode_message(message))
        except OSError:
            with self._lock:
                if self._client_socket is client:
                    self._client_socket = None
                    self._client_address = None
