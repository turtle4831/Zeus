import time

import pytest

from RobotSide.Networking.protocol import (
    CONTROL_TIMEOUT_S,
    ControlMessage,
    HeartbeatMessage,
    LineBuffer,
    TelemetryMessage,
    decode_message,
    encode_message,
)


def test_control_round_trip():
    original = ControlMessage(
        enabled=True,
        lx=0.5,
        ly=-0.25,
        rx=1.0,
        ry=0.0,
        buttons={"cross": True, "circle": False},
        timestamp=100.0,
    )
    decoded = decode_message(encode_message(original))
    assert isinstance(decoded, ControlMessage)
    assert decoded.enabled is True
    assert decoded.lx == 0.5
    assert decoded.ly == -0.25
    assert decoded.buttons["cross"] is True


def test_telemetry_round_trip():
    original = TelemetryMessage(
        robot_state="running",
        subsystems={"feeder": "shooting"},
        battery_voltage=12.4,
        connected=True,
        extra={"loop_time_ms": 5.2},
        timestamp=200.0,
    )
    decoded = decode_message(encode_message(original))
    assert isinstance(decoded, TelemetryMessage)
    assert decoded.robot_state == "running"
    assert decoded.subsystems["feeder"] == "shooting"
    assert decoded.battery_voltage == 12.4


def test_heartbeat_round_trip():
    original = HeartbeatMessage(role="robot", timestamp=50.0)
    decoded = decode_message(encode_message(original))
    assert isinstance(decoded, HeartbeatMessage)
    assert decoded.role == "robot"


def test_line_buffer_splits_multiple_messages():
    control = encode_message(ControlMessage(enabled=True, lx=1.0))
    telemetry = encode_message(TelemetryMessage(connected=True))
    buffer = LineBuffer()
    messages = buffer.feed(control + telemetry)
    assert len(messages) == 2
    assert isinstance(messages[0], ControlMessage)
    assert isinstance(messages[1], TelemetryMessage)


def test_line_buffer_handles_partial_lines():
    control = encode_message(ControlMessage(enabled=True))
    buffer = LineBuffer()
    half = len(control) // 2
    assert buffer.feed(control[:half]) == []
    messages = buffer.feed(control[half:])
    assert len(messages) == 1
    assert messages[0].enabled is True


def test_control_zeroed_is_disabled():
    zeroed = ControlMessage().zeroed()
    assert zeroed.enabled is False
    assert zeroed.lx == 0.0


def test_decode_unknown_type_raises():
    with pytest.raises(ValueError, match="Unknown message type"):
        decode_message('{"type":"unknown"}')


def test_control_timeout_constant_is_reasonable():
    assert 0 < CONTROL_TIMEOUT_S < 1.0
