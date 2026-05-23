import socket
import threading
import time

from RobotSide.Networking.driver_station_server import DriverStationServer
from RobotSide.Networking.protocol import (
    ControlMessage,
    LineBuffer,
    TelemetryMessage,
    decode_message,
    encode_message,
)


def test_server_receives_control_and_returns_telemetry():
    server = DriverStationServer(host="127.0.0.1", port=0)
    server.start()
    port = server.port

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("127.0.0.1", port))
    time.sleep(0.15)

    control = ControlMessage(enabled=True, lx=0.5, ly=-0.5, rx=0.25, timestamp=time.time())
    client.sendall(encode_message(control))
    time.sleep(0.1)

    received = server.get_controls()
    assert received.enabled is True
    assert received.lx == 0.5
    assert received.ly == -0.5

    server.send_telemetry(TelemetryMessage(robot_state="running", battery_voltage=12.0))
    buffer = LineBuffer()
    telemetry = None
    deadline = time.time() + 2.0
    while time.time() < deadline and telemetry is None:
        data = client.recv(4096)
        if data:
            for message in buffer.feed(data):
                if isinstance(message, TelemetryMessage):
                    telemetry = message
        time.sleep(0.05)

    assert telemetry is not None
    assert telemetry.robot_state == "running"
    assert telemetry.battery_voltage == 12.0

    server.stop()
    client.close()


def test_controls_zero_after_timeout():
    server = DriverStationServer(control_timeout_s=0.05)
    server._latest_control = ControlMessage(enabled=True, lx=1.0)
    server._last_control_time = time.time() - 1.0
    controls = server.get_controls()
    assert controls.enabled is False
    assert controls.lx == 0.0
