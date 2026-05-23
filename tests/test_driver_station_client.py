import socket
import threading
import time

from DriverStation.network_client import DriverStationClient
from RobotSide.Networking.driver_station_server import DriverStationServer
from RobotSide.Networking.protocol import ControlMessage, TelemetryMessage


def test_client_server_exchange():
    server = DriverStationServer(host="127.0.0.1", port=0)
    telemetry_events = []

    server.start(
        telemetry_provider=lambda: TelemetryMessage(
            robot_state="test",
            subsystems={"swerve": "ok"},
            battery_voltage=12.1,
        )
    )
    port = server.port

    received_controls = []

    def control_provider():
        msg = ControlMessage(enabled=True, lx=0.1, ly=0.2, rx=0.3)
        received_controls.append(msg)
        return msg

    client = DriverStationClient(host="127.0.0.1", port=port, control_rate_hz=20.0)
    client.connect(
        control_provider=control_provider,
        on_telemetry=telemetry_events.append,
    )

    deadline = time.time() + 3.0
    while time.time() < deadline and not telemetry_events:
        time.sleep(0.05)

    client.disconnect()
    server.stop()

    assert received_controls
    assert server.get_controls().lx == 0.1
    assert telemetry_events
    assert telemetry_events[0].robot_state == "test"
    assert telemetry_events[0].subsystems["swerve"] == "ok"
