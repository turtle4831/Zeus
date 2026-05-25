import enum

from wpimath.controller import PIDController
from RobotSide.Utils.motor import Motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder
from Zeus.RobotSide.Utils.servo import servo

class IntakeState(enum.Enum):
    INTAKE = 1
    OUTTAKE = 2
    IDLE = 3
    STOP = 4
    PIVOT_UP = 5
    

class Intake:
    def __init__(self, IntakeRollerMotorId:int, encoder:absoluteEncoder, IntakePivotServoId:int):
        self.pivotController = PIDController(1,0,0)
        self.motor = Motor(IntakeRollerMotorId, encoder, self.pivotController, pidType=pidTypes.POSITION)
        self.servo = servo(IntakePivotServoId, 0, 180)

        self.state = IntakeState.STOP

    def setSpeed(self, speed):
        self.motor.setSpeed(speed)

    def getSpeed(self):
        return self.motor.getSpeed()

    def setPivotPosition(self, position):
        self.servo.setPosition(position)

    def getPivotPosition(self):
        return self.servo.getPosition()

    def setState(self, state):
        self.state = state

    def changeState(self, state):
        self.setState(state)

    def update(self):
        match (self.state):
            case IntakeState.INTAKE:
                self.motor.setSpeed(1)
                self.servo.setPosition(0)
            case IntakeState.OUTTAKE:
                self.motor.setSpeed(-1)
            case IntakeState.IDLE:
                self.motor.setSpeed(0)
            case IntakeState.STOP:
                self.motor.setSpeed(0)
            case IntakeState.PIVOT_UP:
                self.servo.setPosition(90)
            case _:
                self.motor.setSpeed(0)
                raise ValueError(f"Invalid state: {self.state}")