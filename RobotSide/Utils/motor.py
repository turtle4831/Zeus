import enum
from gpiozero import PWMOutputDevice
import time
from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from wpimath.controller import PIDController

class pidTypes(enum.Enum):
    POSITION = 1
    VELOCITY = 2

class Motor():
    def __init__(self, pin, encoder: absoluteEncoder, pidController: PIDController | PIDController = PIDController(1,0,0), pidType:pidTypes = pidTypes.POSITION):

        self.encoder = encoder
        self.motor = PWMOutputDevice(pin,frequency=50)

        self.pidController = pidController

        self.pidType = pidType

        self.timeElapsed = 0
        if self.encoder is not None:
            self.encoderEnabled = True
        else:
            self.encoderEnabled = False
        
    def setSpeed(self, speed):
        self.motor.value = speed

    def getSpeed(self):
        return self.velocity

    def getPosition(self):
        if not self.encoderEnabled:
            raise ValueError("Encoder not enabled for this motor")

        return self.encoder.getPosition()
        
    def update(self, setpoint):
        if self.encoderEnabled:
            self.encoder.updatePosition()
            if self.pidType == pidTypes.POSITION:
                output = self.pidController.calculate(setpoint, self.encoder.getPosition())
                self.setSpeed(output)
            else:
                elapsed_time = time.time() - self.lastUpdateTime  # calculate time elapsed since last update
                previous_position = self.encoder.getTotalPosition()  # get position before updating
                
                self.encoder.updatePosition()
                current_position = self.encoder.getTotalPosition()  # get updated position
                
                self.lastUpdateTime = time.time()
                self.velocity = (current_position - previous_position) / elapsed_time if elapsed_time > 0 else 0  # calculate velocity
                output = self.pidController.calculate(setpoint, self.velocity)
                self.setSpeed(output)
        else:
            raise ValueError("Encoder not enabled for this motor")

    def initializeESC(self):
        self.setSpeed(1.0)
        time.sleep(2)
        self.setSpeed(0.05)
        time.sleep(2)
        self.setSpeed(0.0)
    

    
motor = Motor
