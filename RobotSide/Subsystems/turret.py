import enum
import math

from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from RobotSide.Utils.motor import Motor, pidTypes
from RobotSide.Utils.pid import PIDController

class TurretState(enum.Enum):
    IDLE = 1
    AIMING = 2
    STOP = 3

class Turret:
    def __init__(self, TurretMotorId: int, encoder: absoluteEncoder, maxDegrees: float = 180):
        if maxDegrees <= 0:
            raise ValueError("maxDegrees must be greater than 0")

        self.maxDegrees = maxDegrees
        self.state = TurretState.STOP
        self.targetAngle = 0
        self.angleController = PIDController(1, 0, 0)
        self.motor = Motor(TurretMotorId, encoder, self.angleController, pidType=pidTypes.POSITION)

    def setState(self, state):
        self.state = state

    def setTargetAngle(self, angle: float):
        self.targetAngle = self._wrapAngle(angle)

    def aimAtPoint(self, currentTurretPosition: list[float], targetPosition: list[float]):
        self._validatePoint(currentTurretPosition, "currentTurretPosition")
        self._validatePoint(targetPosition, "targetPosition")

        deltaX = targetPosition[0] - currentTurretPosition[0]
        deltaY = targetPosition[1] - currentTurretPosition[1]

        if deltaX == 0 and deltaY == 0:
            raise ValueError("targetPosition must be different from currentTurretPosition")

        targetAngle = math.degrees(math.atan2(deltaY, deltaX))
        self.setTargetAngle(targetAngle)

    def getTargetAngle(self):
        return self.targetAngle

    def getCurrentAngle(self):
        return self._wrapAngle(self.motor.encoder.getPosition())

    def setMaxDegrees(self, maxDegrees: float):
        if maxDegrees <= 0:
            raise ValueError("maxDegrees must be greater than 0")

        self.maxDegrees = maxDegrees
        self.targetAngle = self._wrapAngle(self.targetAngle)

    def update(self):
        self.motor.update(self.targetAngle)

    def stop(self):
        self.motor.setSpeed(0)

    def _wrapAngle(self, angle: float):
        fullRange = self.maxDegrees * 2

        while angle > self.maxDegrees:
            angle -= fullRange

        while angle < -self.maxDegrees:
            angle += fullRange

        return angle

    def _validatePoint(self, point: list[float], name: str):
        if len(point) != 2:
            raise ValueError(f"{name} must contain exactly two values: [x, y]")
