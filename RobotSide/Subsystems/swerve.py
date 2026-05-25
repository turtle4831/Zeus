from RobotSide.swerve.swerveKinematics import SwerveKinematics
from RobotSide.Utils.gyro import Gyro
from enum import Enum
import math

class SwerveState(Enum):
    IDLE = 1
    DRIVING = 2
    STOP = 3

class Swerve:
    def __init__(self, front_left, front_right, back_left, back_right, gyro=None):
        self.front_left = front_left
        self.front_right = front_right
        self.back_left = back_left
        self.back_right = back_right
        self.modules = [self.front_left, self.front_right, self.back_left, self.back_right]
        self.kinematics = SwerveKinematics(wheelbase=10, trackwidth=10) # Adjust wheelbase and trackwidth as needed
        self.gyro = gyro if gyro is not None else Gyro()
        self.gyroAngle = 0
        self.state = SwerveState.STOP

    def setState(self, state):
        self.state = state

    def drive(self, yMovement, xMovement, Turn):
        speeds, angles = self.kinematics.normalize(self.kinematics.toSwerveModuleStates(yMovement, xMovement, Turn),maxSpeed=5)
        self._setModuleStates(speeds, angles)

    def fieldCentricDrive(self, yMovement, xMovement, Turn, gyroAngle):
        gyroRadians = math.radians(gyroAngle)
        robotYMovement = yMovement * math.cos(gyroRadians) + xMovement * math.sin(gyroRadians)
        robotXMovement = -yMovement * math.sin(gyroRadians) + xMovement * math.cos(gyroRadians)
        self.drive(robotYMovement, robotXMovement, Turn)

    def _setModuleStates(self, speeds, angles):
        for i in range(4):
            self.modules[i].setState(speeds[i], angles[i])

    def getPose(self):
        return self.kinematics.getPoseEstimation(self.states,self.getGyroAngle())
    
    def getGyroAngle(self):
        self.gyroAngle = self.gyro.getAngle()
        return self.gyroAngle

    def resetPose(self, x, y, theta):
        self.kinematics.setStartingPose(x, y, theta)

    def update(self, gyroAngle):
        self.states = []
        self.gyroAngle = gyroAngle

        for i in range(4):
            self.states.append(self.modules[i].getState())
        