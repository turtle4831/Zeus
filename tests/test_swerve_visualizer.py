import math
import sys
from pathlib import Path

import pytest

ZEUS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ZEUS_ROOT))

from RobotSide.swerve.swerveKinematics import SwerveKinematics
from swerveVisualizer.swerve_visualizer import SwerveVisualizer


def _field_velocity_direction_deg(field_vx: float, field_vy: float) -> float:
    return math.degrees(math.atan2(field_vy, field_vx))


def _screen_direction_deg(field_direction_deg: float) -> float:
    return -field_direction_deg


def _robot_velocity_from_field(field_vx: float, field_vy: float, heading_deg: float):
    heading = math.radians(heading_deg)
    robot_vx = field_vx * math.cos(heading) + field_vy * math.sin(heading)
    robot_vy = -field_vx * math.sin(heading) + field_vy * math.cos(heading)
    return robot_vx, robot_vy


def test_kinematics_forward_and_strafe_module_angles():
    kinematics = SwerveKinematics(0.8, 0.8)

    forward_speeds, forward_angles = kinematics.toSwerveModuleStates(3, 0, 0)
    strafe_speeds, strafe_angles = kinematics.toSwerveModuleStates(0, 2, 0)

    assert forward_speeds == pytest.approx([3, 3, 3, 3])
    assert forward_angles == pytest.approx([0, 0, 0, 0])
    assert strafe_speeds == pytest.approx([2, 2, 2, 2])
    assert strafe_angles == pytest.approx([90, 90, 90, 90])


def test_visualizer_wheel_angle_matches_field_motion():
    visualizer = SwerveVisualizer.__new__(SwerveVisualizer)
    kinematics = SwerveKinematics(0.8, 0.8)
    heading_deg = 0.0

    cases = [
        (0.0, 3.0),
        (3.0, 0.0),
        (2.0, 2.0),
    ]

    for field_vx, field_vy in cases:
        robot_vx, robot_vy = _robot_velocity_from_field(field_vx, field_vy, heading_deg)
        _, module_angles = kinematics.toSwerveModuleStates(robot_vx, robot_vy, 0)
        expected_screen_angle = _screen_direction_deg(
            _field_velocity_direction_deg(field_vx, field_vy)
        )

        for module_angle in module_angles:
            wheel_angle_deg = math.degrees(
                visualizer._module_wheel_angle_rad(heading_deg, module_angle)
            )
            assert wheel_angle_deg == pytest.approx(expected_screen_angle)
