# pyright: reportArgumentType=false, reportAttributeAccessIssue=false

import sys
import types
from pathlib import Path
from typing import Any, cast

import pytest

from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from RobotSide.Utils.beamBreak import BeamBreak
from RobotSide.Utils.motor import Motor, pidTypes


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ZEUS_ROOT = PROJECT_ROOT / "Zeus"

sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(ZEUS_ROOT))


def fake_encoder(position: float = 0) -> absoluteEncoder:
    return cast(absoluteEncoder, FakeEncoder(position))


def fake_beam_break(interrupted: bool = False) -> BeamBreak:
    return cast(BeamBreak, FakeBeamBreak(interrupted))


def as_fake_motor(motor: Motor) -> "FakeMotor":
    return cast(FakeMotor, motor)


class FakeMotor:
    instances: list["FakeMotor"] = []

    def __init__(self, pin, encoder=None, pidController=None, pidType=None):
        self.pin = pin
        self.encoder = encoder
        self.pidController = pidController
        self.pidType = pidType
        self.speed = 0.0
        self.velocity = 0.0
        self.update_calls: list[float] = []
        self.position = 0.0
        FakeMotor.instances.append(self)

    def setSpeed(self, speed: float):
        self.speed = speed
        self.velocity = speed

    def getSpeed(self) -> float:
        return self.speed

    def _uses_velocity_control(self) -> bool:
        if self.pidType is None:
            return False

        pid_value = getattr(self.pidType, "value", self.pidType)
        return pid_value == pidTypes.VELOCITY.value

    def update(self, setpoint: float):
        self.update_calls.append(setpoint)
        if self._uses_velocity_control():
            self.speed = float(setpoint)
            self.velocity = float(setpoint)
        else:
            self.position = setpoint

    def setPosition(self, position: float):
        self.position = position

    def getPosition(self) -> float:
        return self.position


class FakeServo:
    instances = []

    def __init__(self, pin, min_angle=0, max_angle=180, initPosition=0):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.position = initPosition
        FakeServo.instances.append(self)

    def setPosition(self, position):
        self.position = position

    def getPosition(self):
        return self.position


class FakeEncoder:
    def __init__(self, position: float = 0):
        self.position = position
        self.total_position = position

    def getPosition(self) -> float:
        return self.position

    def getTotalPosition(self) -> float:
        return self.total_position

    def updatePosition(self):
        pass


class FakeBeamBreak:
    def __init__(self, interrupted=False):
        self.interrupted = interrupted

    def isInterrupted(self):
        return self.interrupted


@pytest.fixture(autouse=True)
def reset_fakes():
    FakeMotor.instances = []
    FakeServo.instances = []


def test_feeder_updates_motor_for_each_state(monkeypatch):
    from RobotSide.Subsystems import feeder

    monkeypatch.setattr(feeder, "Motor", FakeMotor)

    subsystem = feeder.Feeder(1, fake_encoder(), fake_beam_break(), fake_beam_break())
    assert subsystem.state == feeder.FeederState.STOP
    assert FakeMotor.instances[0].pidType == feeder.pidTypes.VELOCITY

    subsystem.state = feeder.FeederState.SHOOTING
    subsystem.update()
    subsystem.state = feeder.FeederState.INTAKING
    subsystem.update()
    subsystem.state = feeder.FeederState.STOP
    subsystem.update()

    assert as_fake_motor(subsystem.motor).update_calls == [1, -0.5, 0]


def test_feeder_stops_and_raises_for_invalid_state(monkeypatch):
    from RobotSide.Subsystems import feeder

    monkeypatch.setattr(feeder, "Motor", FakeMotor)

    subsystem = feeder.Feeder(1, fake_encoder(), fake_beam_break(), fake_beam_break())
    subsystem.state = "bad"

    with pytest.raises(ValueError):
        subsystem.update()

    assert as_fake_motor(subsystem.motor).speed == 0


def test_feeder_counts_front_and_back_beam_breaks(monkeypatch):
    from RobotSide.Subsystems import feeder

    monkeypatch.setattr(feeder, "Motor", FakeMotor)

    subsystem = feeder.Feeder(
        1,
        fake_encoder(),
        fake_beam_break(interrupted=True),
        fake_beam_break(interrupted=False),
    )

    subsystem.update()
    assert subsystem.getBallCount() == 1

    subsystem.backBeamBreak.interrupted = True
    subsystem.update()
    assert subsystem.getBallCount() == 2


def test_intake_controls_motor_and_pivot_servo(monkeypatch):
    from RobotSide.Subsystems import intake

    monkeypatch.setattr(intake, "Motor", FakeMotor)
    monkeypatch.setattr(intake, "servo", FakeServo)

    subsystem = intake.Intake(2, fake_encoder(), 3)

    subsystem.setSpeed(0.25)
    assert subsystem.getSpeed() == 0.25

    subsystem.setPivotPosition(45)
    assert subsystem.getPivotPosition() == 45

    subsystem.setState(intake.IntakeState.INTAKE)
    subsystem.update()
    motor = as_fake_motor(subsystem.motor)
    servo = cast(FakeServo, subsystem.servo)
    assert motor.speed == 1
    assert servo.position == 0

    subsystem.changeState(intake.IntakeState.OUTTAKE)
    subsystem.update()
    assert motor.speed == -1

    subsystem.changeState(intake.IntakeState.STOP)
    subsystem.update()
    assert motor.speed == 0

    subsystem.changeState(intake.IntakeState.PIVOT_UP)
    subsystem.update()
    assert servo.position == 90


def test_intake_stops_and_raises_for_invalid_state(monkeypatch):
    from RobotSide.Subsystems import intake

    monkeypatch.setattr(intake, "Motor", FakeMotor)
    monkeypatch.setattr(intake, "servo", FakeServo)

    subsystem = intake.Intake(2, fake_encoder(), 3)
    subsystem.state = "bad"

    with pytest.raises(ValueError):
        subsystem.update()

    assert as_fake_motor(subsystem.motor).speed == 0


def test_shooter_delegates_to_shooter_and_hood_motors(monkeypatch):
    from RobotSide.Subsystems import shooter

    monkeypatch.setattr(shooter, "Motor", FakeMotor)

    subsystem = shooter.Shooter(4, fake_encoder(100), 5, fake_encoder(25))

    assert subsystem.state == shooter.ShooterState.STOP
    shooter_motor = as_fake_motor(subsystem.shooterMotor)
    hood_motor = as_fake_motor(subsystem.hoodMotor)
    assert shooter_motor.pidType == shooter.pidTypes.VELOCITY
    assert hood_motor.pidType == shooter.pidTypes.POSITION

    subsystem.setSpeed(0.75)
    assert shooter_motor.update_calls == [0.75]
    assert subsystem.getSpeed() == 0.75
    assert subsystem.atSpeed() is True

    shooter_motor.speed = 0.69
    assert subsystem.atSpeed() is False

    shooter_motor.speed = 0.70
    assert subsystem.atSpeed() is True

    subsystem.setHoodPosition(30)
    assert hood_motor.update_calls == [30]
    assert hood_motor.position == 30
    assert subsystem.getHoodPosition() == 30

    subsystem.setState(shooter.ShooterState.PRE_SHOOTING)
    assert subsystem.state == shooter.ShooterState.PRE_SHOOTING

    assert subsystem.update() is None


def test_swerve_module_sets_drive_and_turn_state():
    from RobotSide.Subsystems.swerveModule import SwerveModule

    drive_motor = FakeMotor(6)
    turn_motor = FakeMotor(7)
    module = SwerveModule(cast(Motor, drive_motor), cast(Motor, turn_motor))

    module.setState(3, 90)

    assert drive_motor.update_calls == [3]
    assert turn_motor.update_calls == [90]
    assert module.getState() == (0, 90)


def test_swerve_drives_modules_and_tracks_pose(monkeypatch):
    fake_module = types.ModuleType("swerveKinematics")

    class FakeKinematics:
        def __init__(self, wheelbase, trackwidth):
            self.wheelbase = wheelbase
            self.trackwidth = trackwidth
            self.starting_pose = None

        def toSwerveModuleStates(self, yMovement, xMovement, Turn):
            return ([yMovement, xMovement, Turn, 4], [10, 20, 30, 40])

        def normalize(self, states, maxSpeed):
            return states

        def setStartingPose(self, x, y, theta):
            self.starting_pose = (x, y, theta)

        def getPoseEstimation(self, states, gyroAngle):
            return (states, gyroAngle)

    from RobotSide.Subsystems import swerve

    monkeypatch.setattr(swerve, "SwerveKinematics", FakeKinematics)

    class FakeSwerveModule:
        def __init__(self):
            self.calls = []

        def setState(self, speed, angle):
            self.calls.append((speed, angle))

        def getState(self):
            return self.calls[-1]

    modules = [FakeSwerveModule() for _ in range(4)]

    subsystem = swerve.Swerve(*modules)
    assert subsystem.state == swerve.SwerveState.STOP

    subsystem.setState(swerve.SwerveState.DRIVING)
    assert subsystem.state == swerve.SwerveState.DRIVING

    subsystem.drive(1, 2, 3)

    assert [module.calls[-1] for module in modules] == [(1, 10), (2, 20), (3, 30), (4, 40)]

    subsystem.update(180)
    assert subsystem.states == [(1, 10), (2, 20), (3, 30), (4, 40)]
    assert subsystem.getPose() == (subsystem.states, 180)

    subsystem.resetPose(1, 2, 90)
    kinematics = cast(Any, subsystem.kinematics)
    assert kinematics.starting_pose == (1, 2, 90)


def test_multiplexer_selects_channels_and_reads_encoder_data(monkeypatch):
    from RobotSide.Subsystems import multiplexer

    class FakeBus:
        def __init__(self, bus_number):
            self.bus_number = bus_number
            self.writes = []

        def write_byte(self, address, value):
            self.writes.append((address, value))

        def read_byte(self, address):
            if address in (0x10, 0x20):
                return 1
            raise OSError("no device")

        def read_byte_data(self, address, register):
            return {0x0E: 0x08, 0x0F: 0x00}[register]

    monkeypatch.setattr(multiplexer.smbus, "SMBus", FakeBus)

    subsystem = multiplexer.Multiplexer(0x70)
    subsystem.select_channel(3)
    assert subsystem.bus.writes[-1] == (0x70, 1 << 3)

    with pytest.raises(ValueError):
        subsystem.select_channel(8)

    subsystem.add_device(2)
    assert subsystem.getEncoderData(2) == 180

    assert subsystem.getAllData()[2] == 180
    assert subsystem.scanDevices() == [0x10, 0x20]


def test_turret_wraps_angles_and_updates_motor(monkeypatch):
    from RobotSide.Subsystems import turret

    monkeypatch.setattr(turret, "Motor", FakeMotor)

    subsystem = turret.Turret(8, fake_encoder(200), maxDegrees=180)
    assert subsystem.state == turret.TurretState.STOP

    subsystem.setState(turret.TurretState.AIMING)
    assert subsystem.state == turret.TurretState.AIMING

    subsystem.setTargetAngle(200)
    assert subsystem.getTargetAngle() == -160

    subsystem.update()
    motor = as_fake_motor(subsystem.motor)
    assert motor.update_calls == [-160]

    assert subsystem.getCurrentAngle() == -160

    subsystem.setTargetAngle(-200)
    assert subsystem.getTargetAngle() == 160

    subsystem.setMaxDegrees(90)
    subsystem.setTargetAngle(100)
    assert subsystem.getTargetAngle() == -80

    subsystem.stop()
    assert motor.speed == 0


def test_turret_aims_at_point(monkeypatch):
    from RobotSide.Subsystems import turret

    monkeypatch.setattr(turret, "Motor", FakeMotor)

    subsystem = turret.Turret(8, fake_encoder(), maxDegrees=180)

    subsystem.aimAtPoint([0.0, 0.0], [1.0, 0.0])
    assert subsystem.getTargetAngle() == 0

    subsystem.aimAtPoint([0.0, 0.0], [0.0, 1.0])
    assert subsystem.getTargetAngle() == 90

    subsystem.aimAtPoint([0.0, 0.0], [0.0, -1.0])
    assert subsystem.getTargetAngle() == -90

    subsystem.aimAtPoint([0.0, 0.0], [-1.0, 0.0])
    assert subsystem.getTargetAngle() == 180


def test_turret_aim_at_point_uses_angle_wrapping(monkeypatch):
    from RobotSide.Subsystems import turret

    monkeypatch.setattr(turret, "Motor", FakeMotor)

    subsystem = turret.Turret(8, fake_encoder(), maxDegrees=90)
    subsystem.aimAtPoint([0.0, 0.0], [-1.0, 0.0])

    assert subsystem.getTargetAngle() == 0


def test_turret_aim_at_point_rejects_invalid_points(monkeypatch):
    from RobotSide.Subsystems import turret

    monkeypatch.setattr(turret, "Motor", FakeMotor)

    subsystem = turret.Turret(8, fake_encoder())

    with pytest.raises(ValueError, match="exactly two values"):
        subsystem.aimAtPoint([0.0], [1.0, 1.0])

    with pytest.raises(ValueError, match="different"):
        subsystem.aimAtPoint([1.0, 1.0], [1.0, 1.0])


def test_turret_rejects_invalid_max_degrees(monkeypatch):
    from RobotSide.Subsystems import turret

    monkeypatch.setattr(turret, "Motor", FakeMotor)

    with pytest.raises(ValueError):
        turret.Turret(8, fake_encoder(), maxDegrees=0)
