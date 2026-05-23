import pytest

from RobotSide.shooting.shootingKinematics import ShootingKinematics


def test_exact_lookup_values_are_returned():
    kinematics = ShootingKinematics()

    assert kinematics.calculateShootingSpeed(0.0) == 5000.0
    assert kinematics.calculateShootingAngle(0.0) == 0.0
    assert kinematics.calculateTimeOfFlight(0.0) == 0.10
    assert kinematics.calculateShootingSpeed(5.0) == 5800.0
    assert kinematics.calculateShootingAngle(5.0) == 5.0
    assert kinematics.calculateTimeOfFlight(5.0) == 0.45


def test_values_between_lookup_points_are_interpolated():
    kinematics = ShootingKinematics()

    assert kinematics.calculateShootingSpeed(1.25) == 5200.0
    assert kinematics.calculateShootingAngle(1.25) == 1.25
    assert kinematics.calculateTimeOfFlight(1.25) == 0.175
    assert kinematics.calculateShootingSpeed(6.25) == 5975.0
    assert kinematics.calculateShootingAngle(6.25) == 6.25
    assert kinematics.calculateTimeOfFlight(6.25) == 0.55


def test_distance_outside_lookup_range_is_clamped():
    kinematics = ShootingKinematics()

    assert kinematics.calculateShootingSpeed(-1.0) == 5000.0
    assert kinematics.calculateShootingAngle(-1.0) == 0.0
    assert kinematics.calculateTimeOfFlight(-1.0) == 0.10
    assert kinematics.calculateShootingSpeed(20.0) == 6500.0
    assert kinematics.calculateShootingAngle(20.0) == 10.0
    assert kinematics.calculateTimeOfFlight(20.0) == 0.85


def test_calculate_shot_returns_speed_angle_and_time_of_flight():
    kinematics = ShootingKinematics()

    assert kinematics.calculateShot(2.5) == (5400.0, 2.5, 0.25)


def test_custom_lookup_table_is_sorted_before_interpolation():
    kinematics = ShootingKinematics(
        lookupTable=[
            (10.0, 6500.0, 10.0, 0.8),
            (0.0, 5000.0, 0.0, 0.1),
            (5.0, 6000.0, 6.0, 0.5),
        ]
    )

    assert kinematics.calculateShot(2.5) == pytest.approx((5500.0, 3.0, 0.3))


def test_lookup_table_requires_at_least_two_points():
    with pytest.raises(ValueError, match="at least two points"):
        ShootingKinematics(lookupTable=[(0.0, 5000.0, 0.0, 0.1)])


def test_lookup_table_rejects_duplicate_distances():
    with pytest.raises(ValueError, match="unique"):
        ShootingKinematics(
            lookupTable=[
                (0.0, 5000.0, 0.0, 0.1),
                (0.0, 5100.0, 1.0, 0.2),
            ]
        )


def test_lookup_table_rejects_values_outside_limits():
    with pytest.raises(ValueError, match="speed"):
        ShootingKinematics(
            lookupTable=[
                (0.0, 4999.0, 0.0, 0.1),
                (1.0, 5100.0, 1.0, 0.2),
            ]
        )


def test_lookup_table_rejects_time_of_flight_outside_limits():
    with pytest.raises(ValueError, match="time of flight"):
        ShootingKinematics(
            lookupTable=[
                (0.0, 5000.0, 0.0, 0.1),
                (1.0, 5100.0, 1.0, 1.1),
            ]
        )
