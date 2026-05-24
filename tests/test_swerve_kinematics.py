import pytest

from RobotSide.swerve.swerveKinematics import SwerveKinematics


def assert_list_close(actual, expected):
    assert actual == pytest.approx(expected)


def test_to_swerve_module_states_forward_motion():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    speeds, angles = kinematics.toSwerveModuleStates(vx=3, vy=0, omega=0)

    assert_list_close(speeds, [3, 3, 3, 3])
    assert_list_close(angles, [0, 0, 0, 0])


def test_to_swerve_module_states_strafe_motion():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    speeds, angles = kinematics.toSwerveModuleStates(vx=0, vy=2, omega=0)

    assert_list_close(speeds, [2, 2, 2, 2])
    assert_list_close(angles, [90, 90, 90, 90])


def test_to_swerve_module_states_rotation_only():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    speeds, angles = kinematics.toSwerveModuleStates(vx=0, vy=0, omega=1)

    assert_list_close(speeds, [2**0.5, 2**0.5, 2**0.5, 2**0.5])
    assert_list_close(angles, [135, 45, -135, -45])


def test_normalize_scales_only_when_above_max_speed():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    speeds, angles = kinematics.normalize(([2, 4, 6, 8], [0, 10, 20, 30]), maxSpeed=4)

    assert_list_close(speeds, [1, 2, 3, 4])
    assert angles == [0, 10, 20, 30]


def test_normalize_does_not_scale_when_under_max_speed():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    speeds, angles = kinematics.normalize(([1, 2, 3, 4], [0, 10, 20, 30]), maxSpeed=5)

    assert speeds == [1, 2, 3, 4]
    assert angles == [0, 10, 20, 30]


def test_optimize_wraps_angle_to_nearest_equivalent_within_90_degrees():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)

    assert kinematics.optimize(45) == 45
    assert kinematics.optimize(135) == -45
    assert kinematics.optimize(-135) == 45
    assert kinematics.optimize(225) == 45


def test_get_velocity_returns_field_relative_x_and_y_velocity():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)
    moduleStates = [(2, 0), (2, 0), (2, 0), (2, 0)]

    assert kinematics.getVelocity(moduleStates, gyroAngle=0) == pytest.approx((2, 0))
    assert kinematics.getVelocity(moduleStates, gyroAngle=90) == pytest.approx((0, 2))


def test_get_velocity_averages_out_rotation_components():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)
    moduleStates = list(zip(*kinematics.toSwerveModuleStates(vx=1, vy=2, omega=1)))

    assert kinematics.getVelocity(moduleStates, gyroAngle=0) == pytest.approx((1, 2))


def test_pose_estimation_integrates_field_relative_velocity():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)
    kinematics.setStartingPose(1, 2, 0)
    moduleStates = [(3, 0), (3, 0), (3, 0), (3, 0)]

    pose = kinematics.getPoseEstimation(moduleStates, gyroAngle=0, deltaTime=0.5)

    assert pose == pytest.approx((2.5, 2, 0))


def test_get_pose_estimation_accepts_speed_and_angle_lists():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)
    speeds, angles = kinematics.toSwerveModuleStates(vx=2, vy=0, omega=0)

    pose = kinematics.getPoseEstimation((speeds, angles), gyroAngle=0, deltaTime=0.5)

    assert pose == pytest.approx((1, 0, 0))


def test_pose_estimation_uses_gyro_angle_for_field_relative_motion():
    kinematics = SwerveKinematics(wheelbase=2, trackwidth=2)
    moduleStates = [(2, 0), (2, 0), (2, 0), (2, 0)]

    pose = kinematics.getPoseEstimation(moduleStates, gyroAngle=90, deltaTime=1.0)

    assert pose == pytest.approx((0, 2, 90))
