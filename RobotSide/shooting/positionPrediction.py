class PositionPrediction:
    """
    Takes current robot position and current robot velocity in a vector 
    and predicts the position of the robot after a given time (tof).
    """
    def __init__(self):
        pass

    def predictPosition(self, currentPosition: tuple[float, float], currentVelocity: tuple[float, float], timeOfFlight: float): #get tof from shooting kinematics
        newPosition = (currentPosition[0] + currentVelocity[0] * timeOfFlight, currentPosition[1] + currentVelocity[1] * timeOfFlight)
        return newPosition
        pass
