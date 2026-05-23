import enum
from wpimath.controller import PIDController
from RobotSide.Utils.motor import motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder

class FeederState(enum.Enum):
    SHOOTING = 1
    INTAKING = 2
    STOP = 3

class Feeder:
    def __init__(self, FeederMotorId:int, encoder:absoluteEncoder):

        self.state = FeederState.STOP

        self.velocityController = PIDController(1,0,0)
        self.motor = motor(FeederMotorId, encoder, self.velocityController, pidType=pidTypes.VELOCITY)

    def setSpeed(self, speed):
        self.motor.setSpeed(speed)

    def getSpeed(self):
        return self.motor.getSpeed()
    
    def setState(self, state):
        self.state = state

    def update(self):
        match (self.state):
            case FeederState.SHOOTING:
                self.motor.update(1)    
            case FeederState.INTAKING:
                self.motor.update(-0.5)
            case FeederState.STOP:
                self.motor.update(0)
            case _:
                self.motor.setSpeed(0)
                raise ValueError(f"Invalid state: {self.state}")    