import enum
import math
from wpimath.controller import PIDController
from RobotSide.Utils.motor import Motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder

class ShooterState(enum.Enum):
    SHOOTING = 1
    PRE_SHOOTING = 2
    IDLE = 3
    STOP = 4
class Shooter:
    def __init__(self, ShooterMotorId:int, shooterEncoder:absoluteEncoder, hoodMotorId:int, hoodEncoder:absoluteEncoder, speedTolerance:float = 0.05):

        self.state = ShooterState.STOP
        self.targetSpeed = 0
        self.speedTolerance = speedTolerance

        self.velocityController = PIDController(1,0,0)
        self.angleController = PIDController(1,0,0)

        self.shooterMotor = Motor(ShooterMotorId, shooterEncoder, self.velocityController, pidType=pidTypes.VELOCITY)
        self.hoodMotor = Motor(hoodMotorId, hoodEncoder, self.angleController, pidType=pidTypes.POSITION)

    def setSpeed(self, speed):
        self.targetSpeed = speed
        self.shooterMotor.setSpeed(speed)

    def getSpeed(self):
        return self.shooterMotor.getSpeed()
    
    def atSpeed(self):
        speedDifference = abs(self.getSpeed() - self.targetSpeed)
        return speedDifference <= self.speedTolerance or math.isclose(speedDifference, self.speedTolerance)

    def setHoodPosition(self, position):
        self.hoodMotor.setPosition(position)

    def getHoodPosition(self):
        return self.hoodMotor.getPosition()

    def update(self):
        #implement shooter kinematics
        pass