import enum
from wpimath.controller import PIDController
from RobotSide.Utils.motor import Motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder
from Zeus.RobotSide.Utils.beamBreak import BeamBreak

class FeederState(enum.Enum):
    SHOOTING = 1
    INTAKING = 2
    IDLE = 3
    STOP = 4

class Feeder:
    def __init__(self, FeederMotorId:int, encoder:absoluteEncoder, frontBeamBreak:BeamBreak,middleBeamBreak:BeamBreak, backBeamBreak:BeamBreak):

        self.state = FeederState.STOP

        self.velocityController = PIDController(1,0,0)
        self.motor = Motor(FeederMotorId, encoder, self.velocityController, pidType=pidTypes.VELOCITY)

        self.frontBeamBreak = frontBeamBreak
        self.middleBeamBreak = middleBeamBreak
        self.backBeamBreak = backBeamBreak

    def setSpeed(self, speed):
        self.motor.setSpeed(speed)

    def getSpeed(self):
        return self.motor.getSpeed()
    
    def setState(self, state):
        self.state = state

    def isFrontBeamBreakInterrupted(self):
        return self.frontBeamBreak.isInterrupted()
    
    def isBackBeamBreakInterrupted(self):
        return self.backBeamBreak.isInterrupted()
    def isMiddleBeamBreakInterrupted(self):
        return self.middleBeamBreak.isInterrupted()

    def getBallCount(self):
        return self.ballCount
    def setBallCount(self, ballCount):
        self.ballCount = ballCount
    def resetBallCount(self):
        self.ballCount = 0
        
    def update(self):
        self.ballCount = int(self.isFrontBeamBreakInterrupted()) + int(self.isBackBeamBreakInterrupted()) + int(self.isMiddleBeamBreakInterrupted())

        match (self.state):
            case FeederState.SHOOTING:
                self.motor.update(1)    
            case FeederState.INTAKING:
                self.motor.update(-0.5)
            case FeederState.STOP:
                self.motor.update(0)
            case FeederState.IDLE:
                self.motor.update(0)
            case _:
                self.motor.setSpeed(0)
                raise ValueError(f"Invalid state: {self.state}")    