from typing import Any


class _FallbackPIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._previous_error = 0.0

    def calculate(self, measurement: float, setpoint: float = 0.0) -> float:
        error = setpoint - measurement
        self._integral += error
        derivative = error - self._previous_error
        self._previous_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


try:
    from wpimath.controller import PIDController as _RobotPyPIDController
except ModuleNotFoundError:
    _RobotPyPIDController = None

PIDController: Any = _RobotPyPIDController or _FallbackPIDController
