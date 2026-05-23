from gpiozero import AngularServo


class servo:
    def __init__(self, pin: int, min_angle: float = 0, max_angle: float = 180, initPosition:float = 0):
        self._min_angle = min_angle
        self._max_angle = max_angle
        self._device = AngularServo(
            pin,
            min_angle=min_angle,
            max_angle=max_angle,
        )
        self._position = initPosition
        self.setPosition(initPosition)

    def setPosition(self, position: float) -> None:
        clamped = max(self._min_angle, min(position, self._max_angle))
        self._position = clamped
        self._device.angle = clamped

    def getPosition(self) -> float:
        angle = self._device.angle
        if angle is None:
            return self._position
        self._position = angle
        return angle
