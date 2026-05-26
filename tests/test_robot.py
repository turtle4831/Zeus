import time

from robot import Robot
from RobotSide.Networking.protocol import INITIALIZE_ESC_BUTTON, ControlMessage


class FakeStateMachine:
    def __init__(self):
        self.initialize_calls = 0

    def initialize_all_escs(self):
        self.initialize_calls += 1


def test_robot_applies_controls_when_enabled():
    robot = Robot(state_machine=FakeStateMachine())
    robot.driver_station._latest_control = ControlMessage(
        enabled=True, lx=0.5, ly=-0.25, rx=0.1
    )
    robot.driver_station._last_control_time = time.time()
    robot.periodic()
    assert robot.robot_state == "teleop"
    assert "drive" in robot.subsystem_states


def test_robot_idle_when_disconnected_and_no_controls():
    robot = Robot(state_machine=FakeStateMachine())
    robot.periodic()
    assert robot.robot_state == "idle"


def test_robot_initializes_esc_when_disabled_and_button_pressed():
    state_machine = FakeStateMachine()
    robot = Robot(state_machine=state_machine)

    robot.driver_station._latest_control = ControlMessage(
        enabled=False,
        buttons={INITIALIZE_ESC_BUTTON: True},
    )
    robot.driver_station._last_control_time = time.time()
    robot.periodic()

    assert state_machine.initialize_calls == 1
    assert robot.subsystem_states["esc_init"] == "completed"


def test_robot_rejects_esc_init_when_controls_enabled():
    state_machine = FakeStateMachine()
    robot = Robot(state_machine=state_machine)

    robot.driver_station._latest_control = ControlMessage(
        enabled=True,
        buttons={INITIALIZE_ESC_BUTTON: True},
    )
    robot.driver_station._last_control_time = time.time()
    robot.periodic()

    assert state_machine.initialize_calls == 0
    assert robot.subsystem_states["esc_init"] == "rejected: controls enabled"


def test_robot_esc_init_only_on_rising_edge():
    state_machine = FakeStateMachine()
    robot = Robot(state_machine=state_machine)

    control = ControlMessage(
        enabled=False,
        buttons={INITIALIZE_ESC_BUTTON: True},
    )
    robot.driver_station._latest_control = control
    robot.driver_station._last_control_time = time.time()

    robot.periodic()
    robot.periodic()

    assert state_machine.initialize_calls == 1


def test_robot_builds_telemetry():
    robot = Robot(state_machine=FakeStateMachine())
    robot.robot_state = "teleop"
    robot.battery_voltage = 12.5
    robot.subsystem_states["feeder"] = "stop"
    telemetry = robot._build_telemetry()
    assert telemetry.robot_state == "teleop"
    assert telemetry.battery_voltage == 12.5
    assert telemetry.subsystems["feeder"] == "stop"
