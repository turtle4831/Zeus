import math


class SwerveKinematics:
    def __init__(self, wheelbase, trackwidth):
        self.wheelbase = wheelbase
        self.trackwidth = trackwidth
        self.radius = (wheelbase ** 2 + trackwidth ** 2) ** 0.5

        self.poseEstimation = (0, 0, 0) #x, y, theta
        self.moduleLocations = [
            (wheelbase / 2, trackwidth / 2),   # Front Left
            (wheelbase / 2, -trackwidth / 2),  # Front Right
            (-wheelbase / 2, trackwidth / 2),  # Back Left
            (-wheelbase / 2, -trackwidth / 2), # Back Right
        ]

    def setStartingPose(self, x, y, theta):
        self.poseEstimation = (x, y, theta)

    def toSwerveModuleStates(self, vx, vy, omega):
        speeds = []
        angles = []

        for moduleX, moduleY in self.moduleLocations:
            moduleVx = vx - omega * moduleY
            moduleVy = vy + omega * moduleX

            speeds.append(math.hypot(moduleVx, moduleVy))
            angles.append(math.degrees(math.atan2(moduleVy, moduleVx)))

        return speeds, angles
    
    def optimize(self,currentAngle):
        currentAngle = self._wrapAngle(currentAngle)
        if currentAngle > 90:
            currentAngle -= 180
        elif currentAngle < -90:
            currentAngle += 180

        return currentAngle
    
    def normalize(self, moduleStates, maxSpeed: float):
        speeds, angles = moduleStates
        highestSpeed = max(abs(speed) for speed in speeds) if speeds else 0

        if highestSpeed > maxSpeed and highestSpeed > 0:
            scale = maxSpeed / highestSpeed
            speeds = [speed * scale for speed in speeds]

        return speeds, angles
    
    def getVelocity(self, moduleStates, gyroAngle=0):
        robotVx, robotVy = self._getRobotRelativeVelocity(moduleStates)

        xVelocity = robotVx * math.cos(math.radians(gyroAngle)) - robotVy * math.sin(math.radians(gyroAngle))
        yVelocity = robotVx * math.sin(math.radians(gyroAngle)) + robotVy * math.cos(math.radians(gyroAngle))

        return xVelocity, yVelocity

    def getPoseEstimation(self, moduleStates, gyroAngle, deltaTime=1.0):
        xVelocity, yVelocity = self.getVelocity(moduleStates, gyroAngle)

        x_global = self.poseEstimation[0] + xVelocity * deltaTime
        y_global = self.poseEstimation[1] + yVelocity * deltaTime

        self.poseEstimation = (x_global, y_global, gyroAngle)
        return self.poseEstimation

    def _coerce_module_states(self, moduleStates):
        if (
            len(moduleStates) == 2
            and isinstance(moduleStates[0], (list, tuple))
            and isinstance(moduleStates[1], (list, tuple))
            and not isinstance(moduleStates[0][0], (list, tuple))
        ):
            speeds, angles = moduleStates
            return list(zip(speeds, angles))

        return list(moduleStates)

    def _getRobotRelativeVelocity(self, moduleStates):
        moduleVelocityComponents = []

        for speed, angle in self._coerce_module_states(moduleStates):
            vi_x = speed * math.cos(math.radians(angle))
            vi_y = speed * math.sin(math.radians(angle))
            moduleVelocityComponents.append((vi_x, vi_y))

        avg_vx = sum(comp[0] for comp in moduleVelocityComponents) / len(moduleVelocityComponents) if moduleVelocityComponents else 0
        avg_vy = sum(comp[1] for comp in moduleVelocityComponents) / len(moduleVelocityComponents) if moduleVelocityComponents else 0

        return avg_vx, avg_vy

    def _wrapAngle(self, angle):
        return ((angle + 180) % 360) - 180
