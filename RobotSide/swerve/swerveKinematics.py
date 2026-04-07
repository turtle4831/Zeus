from math import atan2
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
        A = vx - omega * (self.wheelbase / self.radius)
        B = vx + omega * (self.wheelbase / self.radius)
        C = vy - omega * (self.trackwidth / self.radius)
        D = vy + omega * (self.trackwidth / self.radius)

        speeds = [ #multiply the speeds by 95% meter per second of the robot
            (B ** 2 + D ** 2) ** 0.5,
            (B ** 2 + C ** 2) ** 0.5,
            (A ** 2 + D ** 2) ** 0.5,
            (A ** 2 + C ** 2) ** 0.5,
        ]

        angles = [
            atan2(B, D) * 180 / 3.141592653589793,
            atan2(B, C) * 180 / 3.141592653589793,
            atan2(A, D) * 180 / 3.141592653589793,
            atan2(A, C) * 180 / 3.141592653589793,
        ]

        return speeds, angles
    
    def optimize(self,currentAngle):
        if abs(currentAngle) > 90:
           
            if currentAngle < 0:
                currentAngle += 180
            else:
                currentAngle -= 180
        
        return currentAngle
    
    def normalize(self, moduleStates, maxSpeed: float):
        normalizedSpeeds = []
        normalizedAngles = []

        unnormalizedMaxSpeed = moduleStates[0]
        unnormalizedAngles = moduleStates[1]

        for i in range(4):
            if unnormalizedAngles[i] < 0:
                optimizedAngle = self.optimize(unnormalizedAngles[i])
                normalizedSpeeds.append(-unnormalizedMaxSpeed[i] * maxSpeed)
            else:
                optimizedAngle = self.optimize(unnormalizedAngles[i])
                normalizedSpeeds.append(unnormalizedMaxSpeed[i] * maxSpeed)

            normalizedAngles.append(optimizedAngle)

        return normalizedSpeeds, normalizedAngles
    
    def getPoseEstimation(self, moduleStates, gryoAngle):
        moduleVelocityComponents = []

        for module in moduleStates:
            vi_x = module[0] * math.cos(math.radians(module[1]))
            vi_y = module[0] * math.sin(math.radians(module[1]))
            moduleVelocityComponents.append((vi_x, vi_y))

        avg_vx = sum([comp[0] for comp in moduleVelocityComponents]) / len(moduleVelocityComponents) if moduleVelocityComponents else 0
        avg_vy = sum([comp[1] for comp in moduleVelocityComponents]) / len(moduleVelocityComponents) if moduleVelocityComponents else 0

        x_global = self.poseEstimation[0] + avg_vx * math.cos(math.radians(gryoAngle)) - avg_vy * math.sin(math.radians(gryoAngle))
        y_global = self.poseEstimation[1] + avg_vx * math.sin(math.radians(gryoAngle)) + avg_vy * math.cos(math.radians(gryoAngle))

        self.poseEstimation = (x_global, y_global, gryoAngle)
        return self.poseEstimation

