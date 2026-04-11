from swerveKinematics import SwerveKinematics


class Swerve:
    def __init__(self, front_left, front_right, back_left, back_right):
        self.front_left = front_left
        self.front_right = front_right
        self.back_left = back_left
        self.back_right = back_right
        self.modules = [self.front_left, self.front_right, self.back_left, self.back_right]
        self.kinematics = SwerveKinematics(wheelbase=10, trackwidth=10) # Adjust wheelbase and trackwidth as needed


    def drive(self, yMovement, xMovement, Turn):
        speeds, angles = self.kinematics.normalize(self.kinematics.toSwerveModuleStates(yMovement, xMovement, Turn),maxSpeed=5)

        for i in range(4):
            self.modules[i].setState(speeds[i], angles[i])

    def getPose(self):
        return self.kinematics.getPoseEstimation(self.states,self.gyroAngle)
    def resetPose(self, x, y, theta):
        self.kinematics.setStartingPose(x, y, theta)

    def update(self, gyroAngle):
        self.states = []
        self.gyroAngle = gyroAngle

        for i in range(4):
            self.states.append(self.modules[i].getState())
        