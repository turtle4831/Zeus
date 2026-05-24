class ShootingKinematics:
    def __init__(self, lookupTable: list[tuple[float, float, float, float]] | None = None):
        # All of these constants need to be calibrated on the real robot.
        self.maxShootingSpeed = 6500  # rpm
        self.minShootingSpeed = 5000  # rpm
        self.maxShootingAngle = 45  # degrees
        self.minShootingAngle = 10 # degrees
        self.maxShootingDistance = 10  # meters
        self.minShootingDistance = 0  # meters
        self.maxShootingHeight = 1  # meters
        self.minShootingHeight = 0  # meters
        self.maxTimeOfFlight = 1.0  # seconds
        self.minTimeOfFlight = 0.0  # seconds
        self.targetHeight = 0.5  # meters

        self.lookupTable = lookupTable or [
            # distance meters, shooter speed rpm, hood angle degrees, time of flight seconds
            (0.0, 5000.0, 10.0, 0.10),
            (2.5, 5400.0, 12.5, 0.25),
            (5.0, 5800.0, 15.0, 0.45),
            (7.5, 6150.0, 17.5, 0.65),
            (10.0, 6500.0, 40.0, 0.85),
        ]
        self.lookupTable.sort(key=lambda row: row[0])
        self._validateLookupTable()

    def calculateShootingSpeed(self, distance: float) -> float:
        return self._interpolate(distance, valueIndex=1)

    def calculateShootingAngle(self, distance: float) -> float:
        return self._interpolate(distance, valueIndex=2)

    def calculateTimeOfFlight(self, distance: float) -> float:
        return self._interpolate(distance, valueIndex=3)

    def calculateShot(self, distance: float) -> tuple[float, float, float]:
        return (
            self.calculateShootingSpeed(distance),
            self.calculateShootingAngle(distance),
            self.calculateTimeOfFlight(distance),
        )

    def _interpolate(self, distance: float, valueIndex: int) -> float:
        if distance <= self.lookupTable[0][0]:
            return self.lookupTable[0][valueIndex]

        if distance >= self.lookupTable[-1][0]:
            return self.lookupTable[-1][valueIndex]

        for lower, upper in zip(self.lookupTable, self.lookupTable[1:]):
            lowerDistance = lower[0]
            upperDistance = upper[0]

            if lowerDistance <= distance <= upperDistance:
                ratio = (distance - lowerDistance) / (upperDistance - lowerDistance)
                return lower[valueIndex] + ratio * (upper[valueIndex] - lower[valueIndex])

        return self.lookupTable[-1][valueIndex]

    def _validateLookupTable(self) -> None:
        if len(self.lookupTable) < 2:
            raise ValueError("lookupTable must contain at least two points")

        previousDistance = None
        for distance, speed, angle, timeOfFlight in self.lookupTable:
            if distance < self.minShootingDistance or distance > self.maxShootingDistance:
                raise ValueError("lookupTable distance is outside the configured range")
            if speed < self.minShootingSpeed or speed > self.maxShootingSpeed:
                raise ValueError("lookupTable speed is outside the configured range")
            if angle < self.minShootingAngle or angle > self.maxShootingAngle:
                raise ValueError("lookupTable angle is outside the configured range")
            if timeOfFlight < self.minTimeOfFlight or timeOfFlight > self.maxTimeOfFlight:
                raise ValueError("lookupTable time of flight is outside the configured range")
            if previousDistance is not None and distance <= previousDistance:
                raise ValueError("lookupTable distances must be unique")
            previousDistance = distance