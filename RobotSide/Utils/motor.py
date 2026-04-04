from json import encoder

from gpiozero import PWMOutputDevice
import time

from simple_pid import PID

from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from RobotSide.Utils.baseEncoder import baseEncoder

class motor():
    def __init__(self, pin, encoder: baseEncoder| None = None, pidController:PID | PID = PID(1,0,0,0)):

        self.encoder = encoder
        self.motor = PWMOutputDevice(pin,frequency=50)

        self.pidController = pidController


        if self.encoder is not None:
            self.encoderEnabled = True
        else:
            self.encoderEnabled = False
        
    def setSpeed(self, speed):
        self.motor.value = speed

    

    def initlizeESC(self):
        self.setSpeed(1.0)
        time.sleep(2)
        self.setSpeed(0.05)
        time.sleep(2)
        self.setSpeed(0.0)
    

    
        

    