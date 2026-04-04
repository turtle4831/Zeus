from gpiozero import PWMOutputDevice
import time

class motor():
    def __init__(self, pin):
        self.motor = PWMOutputDevice(pin,frequency=50)
    
    def setSpeed(self, speed):
        self.motor.value = speed

    def initlizeESC(self):
        self.setSpeed(1.0)
        time.sleep(2)
        self.setSpeed(0.05)
        time.sleep(2)
        self.setSpeed(0.0)
    

    
        

    