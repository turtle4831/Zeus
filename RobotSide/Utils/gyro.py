class Gyro:
    def __init__(self, sensor=None):
        self._sensor = sensor
        self._angle_offset = 0.0
        self._last_error = ""

        if self._sensor is None:
            self._sensor = self._create_bno055()

    @property
    def available(self):
        return self._sensor is not None

    @property
    def last_error(self):
        return self._last_error

    def getAngle(self):
        if self._sensor is None:
            return 0.0

        heading = self._heading_from_euler(self._sensor.euler)
        if heading is None:
            return 0.0

        return self._wrapAngle(heading - self._angle_offset)

    def reset(self):
        if self._sensor is None:
            self._angle_offset = 0.0
            return

        heading = self._heading_from_euler(self._sensor.euler)
        if heading is not None:
            self._angle_offset = heading

    def _heading_from_euler(self, euler):
        if euler is None:
            return None

        yaw = euler[0]
        if yaw is None:
            return None

        return float(yaw)

    def _create_bno055(self):
        try:
            import board
            import adafruit_bno055

            return adafruit_bno055.BNO055_I2C(board.I2C())
        except Exception as exc:
            self._last_error = f"Adafruit 9-axis IMU unavailable: {exc}"
            return None

    def _wrapAngle(self, angle):
        return ((angle + 180) % 360) - 180
