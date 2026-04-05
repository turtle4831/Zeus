from json import encoder

from gpiozero import PWMOutputDevice
import time

from simple_pid import PID

from RobotSide.Utils.absoluteEncoder import absoluteEncoder

class motor():
    def __init__(self, pin, encoder: absoluteEncoder, pidController:PID | PID = PID(1,0,0,0)):

        self.encoder = encoder
        self.motor = PWMOutputDevice(pin,frequency=50)

        self.pidController = pidController


        if self.encoder is not None:
            self.encoderEnabled = True
        else:
            self.encoderEnabled = False
        
    def setSpeed(self, speed):
        self.motor.value = speed

    def setSetpoint(self, setpoint):
        self.pidController.setpoint = setpoint

    def update(self):
        if self.encoderEnabled:
            output = self.pidController(self.encoder.getPosition())
            self.setSpeed(output)
        else:
            raise ValueError("Encoder not enabled for this motor")

    def initializeESC(self):
        self.setSpeed(1.0)
        time.sleep(2)
        self.setSpeed(0.05)
        time.sleep(2)
        self.setSpeed(0.0)
    

    
        

    