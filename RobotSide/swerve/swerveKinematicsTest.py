import time

from swerveKinematics import SwerveKinematics


swerveDrive = SwerveKinematics(10, 10)

# Test 1: Straight forward
speeds, angles = swerveDrive.normalize(swerveDrive.toSwerveModuleStates(2, 0, 0.5), 15)
print("Test 1 - Straight Forward")
print("Speeds:", speeds)
print("Angles:", angles)

for i in range(1):
    print(swerveDrive.getPoseEstimation((speeds, angles), 0))