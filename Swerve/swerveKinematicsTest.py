from swerveKinematics import SwerveKinematics


swerveDrive = SwerveKinematics(10, 10)

# Test 1: Straight forward
speeds, angles = swerveDrive.normalize(swerveDrive.toSwerveModuleStates(0, 0, 1), 1)
print("Test 1 - Straight Forward")
print("Speeds:", speeds)
print("Angles:", angles)