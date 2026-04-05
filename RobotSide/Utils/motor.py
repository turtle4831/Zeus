import enum
from json import encoder
from gpiozero import PWMOutputDevice
import time
from simple_pid import PID
from RobotSide.Utils.absoluteEncoder import absoluteEncoder

class pidTypes(enum.Enum):
    POSITION = 1
    VELOCITY = 2

class motor():
    def __init__(self, pin, encoder: absoluteEncoder, pidController:PID | PID = PID(1,0,0,0), pidType:pidTypes = pidTypes.POSITION):

        self.encoder = encoder
        self.motor = PWMOutputDevice(pin,frequency=50)

        self.pidController = pidController

        self.pidType = pidType

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
            if self.pidType == pidTypes.POSITION:
                output = self.pidController(self.encoder.getPosition())
                self.setSpeed(output)
            else:
                #TODO: implement velocity control
                pass
        else:
            raise ValueError("Encoder not enabled for this motor")

    def initializeESC(self):
        self.setSpeed(1.0)
        time.sleep(2)
        self.setSpeed(0.05)
        time.sleep(2)
        self.setSpeed(0.0)
    

    
