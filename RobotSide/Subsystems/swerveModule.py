from RobotSide.Utils.motor import Motor

__all__ = ["SwerveModule"]


class SwerveModule:
    def __init__(self, drive_motor: Motor, turn_motor: Motor):
        self.drive_motor = drive_motor
        self.turn_motor = turn_motor
       

    def set_speed(self, speed):
        #PLEASE REMEMBER TO SET THE DRIVE MOTOR TO VELOCITY CONTROL MODE IN THE ROBOT CLASS
        self.drive_motor.update(speed)

    def set_angle(self, angle):
        self.turn_motor.update(angle)

    def setState(self, speed, angle):
        self.set_speed(speed)
        self.set_angle(angle)

    def getState(self):
        return self.drive_motor.getSpeed(), self.turn_motor.getPosition()

