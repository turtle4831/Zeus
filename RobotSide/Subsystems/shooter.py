import enum
from wpimath.controller import PIDController
from RobotSide.Utils.motor import motor, pidTypes
from Zeus.RobotSide.Utils.absoluteEncoder import absoluteEncoder

class ShooterState(enum.Enum):
    SHOOTING = 1
    PRE_SHOOTING = 2
    IDLING = 3
    STOP = 4
class Shooter:
    def __init__(self, ShooterMotorId:int, shooterEncoder:absoluteEncoder, hoodMotorId:int, hoodEncoder:absoluteEncoder):

        self.state = ShooterState.STOP

        self.velocityController = PIDController(1,0,0)
        self.angleController = PIDController(1,0,0)

        self.shooterMotor = motor(ShooterMotorId, shooterEncoder, self.velocityController, pidType=pidTypes.VELOCITY)
        self.hoodMotor = motor(hoodMotorId, hoodEncoder, self.angleController, pidType=pidTypes.POSITION)

    def setSpeed(self, speed):
        self.shooterMotor.setSpeed(speed)

    def getSpeed(self):
        return self.shooterMotor.getSpeed()

    def setHoodPosition(self, position):
        self.hoodMotor.setPosition(position)

    def getHoodPosition(self):
        return self.hoodMotor.getPosition()

    def update(self):
        #implement shooter kinematics
        pass