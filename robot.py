import time

from RobotSide.Networking.driver_station_server import DriverStationServer
from RobotSide.Networking.protocol import TelemetryMessage


class Robot:
    def __init__(self):
        self.totalTime = 0
        self.robot_state = "idle"
        self.battery_voltage: float | None = None
        self.subsystem_states: dict[str, str] = {}
        self.driver_station = DriverStationServer()

    def init(self):
        self.driver_station.start(telemetry_provider=self._build_telemetry)

    def periodic(self):
        start = time.time()

        controls = self.driver_station.get_controls()
        if controls.enabled:
            self.robot_state = "teleop"
            self._apply_controls(controls)
        elif self.driver_station.is_connected():
            self.robot_state = "connected"
        else:
            self.robot_state = "idle"

        end = time.time()
        self.totalTime += end - start

    def _apply_controls(self, controls) -> None:
        """Apply driver station controls to robot subsystems."""
        self.subsystem_states["drive"] = (
            f"lx={controls.lx:.2f}, ly={controls.ly:.2f}, rx={controls.rx:.2f}"
        )

    def _build_telemetry(self) -> TelemetryMessage:
        return TelemetryMessage(
            robot_state=self.robot_state,
            subsystems=dict(self.subsystem_states),
            battery_voltage=self.battery_voltage,
            connected=self.driver_station.is_connected(),
            extra={"loop_time_s": self.totalTime},
        )

    def shutdown(self):
        self.driver_station.stop()


def main():
    robot = Robot()
    robot.init()
    loop_period_s = 0.02
    try:
        while True:
            loop_start = time.time()
            robot.periodic()
            elapsed = time.time() - loop_start
            sleep_time = loop_period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
