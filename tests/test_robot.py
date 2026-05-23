import time

from robot import Robot
from RobotSide.Networking.protocol import ControlMessage


def test_robot_applies_controls_when_enabled():
    robot = Robot()
    robot.driver_station._latest_control = ControlMessage(
        enabled=True, lx=0.5, ly=-0.25, rx=0.1
    )
    robot.driver_station._last_control_time = time.time()
    robot.periodic()
    assert robot.robot_state == "teleop"
    assert "drive" in robot.subsystem_states


def test_robot_idle_when_disconnected_and_no_controls():
    robot = Robot()
    robot.periodic()
    assert robot.robot_state == "idle"


def test_robot_builds_telemetry():
    robot = Robot()
    robot.robot_state = "teleop"
    robot.battery_voltage = 12.5
    robot.subsystem_states["feeder"] = "stop"
    telemetry = robot._build_telemetry()
    assert telemetry.robot_state == "teleop"
    assert telemetry.battery_voltage == 12.5
    assert telemetry.subsystems["feeder"] == "stop"
