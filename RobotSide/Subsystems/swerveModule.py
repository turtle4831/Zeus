from wpimath.controller import PIDController

from RobotSide.Utils.motor import motor


class Swerve:
    def __init__(self, drive_motor:motor, turn_motor:motor, encoderId):
        self.drive_motor = drive_motor
        self.turn_motor = turn_motor
        self.encoder = encoderId

    def set_speed(self, speed):
        #PLEASE REMEMBER TO SET THE DRIVE MOTOR TO VELOCITY CONTROL MODE IN THE ROBOT CLASS
        self.drive_motor.update(speed)

    def set_angle(self, angle):
        self.turn_motor.update(angle)

    def setState(self, speed, angle):
        self.set_speed(speed)
        self.set_angle(angle)

    def getState(self):
        return self.drive_motor.getSpeed(), self.encoder.getPosition()

