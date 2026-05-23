"""Newline-delimited JSON protocol for driver station <-> robot communication."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class MessageType(str, Enum):
    CONTROL = "control"
    TELEMETRY = "telemetry"
    HEARTBEAT = "heartbeat"


DEFAULT_PORT = 5800
CONTROL_TIMEOUT_S = 0.25


@dataclass
class ControlMessage:
    enabled: bool = False
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    buttons: dict[str, bool] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": MessageType.CONTROL.value,
            "timestamp": self.timestamp,
            "enabled": self.enabled,
            "axes": {"lx": self.lx, "ly": self.ly, "rx": self.rx, "ry": self.ry},
            "buttons": self.buttons,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ControlMessage:
        axes = data.get("axes", {})
        return cls(
            enabled=bool(data.get("enabled", False)),
            lx=float(axes.get("lx", 0.0)),
            ly=float(axes.get("ly", 0.0)),
            rx=float(axes.get("rx", 0.0)),
            ry=float(axes.get("ry", 0.0)),
            buttons={str(k): bool(v) for k, v in data.get("buttons", {}).items()},
            timestamp=float(data.get("timestamp", time.time())),
        )

    def zeroed(self) -> ControlMessage:
        return ControlMessage(enabled=False, timestamp=time.time())


@dataclass
class TelemetryMessage:
    robot_state: str = "idle"
    subsystems: dict[str, Any] = field(default_factory=dict)
    battery_voltage: float | None = None
    connected: bool = False
    extra: dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "type": MessageType.TELEMETRY.value,
            "timestamp": self.timestamp,
            "robot_state": self.robot_state,
            "subsystems": self.subsystems,
            "connected": self.connected,
            "extra": self.extra,
        }
        if self.battery_voltage is not None:
            payload["battery_voltage"] = self.battery_voltage
        return payload

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> TelemetryMessage:
        return cls(
            robot_state=str(data.get("robot_state", "idle")),
            subsystems=dict(data.get("subsystems", {})),
            battery_voltage=data.get("battery_voltage"),
            connected=bool(data.get("connected", False)),
            extra=dict(data.get("extra", {})),
            timestamp=float(data.get("timestamp", time.time())),
        )


@dataclass
class HeartbeatMessage:
    role: str = "driver_station"
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": MessageType.HEARTBEAT.value,
            "timestamp": self.timestamp,
            "role": self.role,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HeartbeatMessage:
        return cls(
            role=str(data.get("role", "unknown")),
            timestamp=float(data.get("timestamp", time.time())),
        )


Message = ControlMessage | TelemetryMessage | HeartbeatMessage


def encode_message(message: Message) -> bytes:
    return (json.dumps(message.to_dict(), separators=(",", ":")) + "\n").encode("utf-8")


def decode_message(line: str | bytes) -> Message:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    data = json.loads(line.strip())
    msg_type = data.get("type")
    if msg_type == MessageType.CONTROL.value:
        return ControlMessage.from_dict(data)
    if msg_type == MessageType.TELEMETRY.value:
        return TelemetryMessage.from_dict(data)
    if msg_type == MessageType.HEARTBEAT.value:
        return HeartbeatMessage.from_dict(data)
    raise ValueError(f"Unknown message type: {msg_type}")


class LineBuffer:
    """Accumulates socket bytes and yields complete newline-delimited lines."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Message]:
        self._buffer.extend(data)
        messages: list[Message] = []
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                break
            line = bytes(self._buffer[:newline])
            del self._buffer[: newline + 1]
            if line.strip():
                messages.append(decode_message(line))
        return messages
