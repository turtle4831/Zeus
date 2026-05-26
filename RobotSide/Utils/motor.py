import enum
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from gpiozero import PWMOutputDevice
else:
    try:
        from gpiozero import PWMOutputDevice
    except ModuleNotFoundError:
        class PWMOutputDevice:
            value = 0.0

            def __init__(self, *args, **kwargs):
                raise ModuleNotFoundError("No module named 'gpiozero'")

import time
from RobotSide.Utils.absoluteEncoder import absoluteEncoder
from RobotSide.Utils.pid import PIDController

class pidTypes(enum.Enum):
    POSITION = 1
    VELOCITY = 2


DEFAULT_ESC_STEPS: list[tuple[float, float]] = [
    (1.0, 2.0),
    (0.05, 2.0),
    (0.0, 0.0),
]


class initializeESC:
    """Parent class for per-motor ESC initialization routines."""

    def run(self, motor: "Motor") -> None:
        raise NotImplementedError


class SteppedInitializeESC(initializeESC):
    def __init__(self, steps: list[tuple[float, float]]):
        self.steps = steps

    def run(self, motor: "Motor") -> None:
        for power, duration_s in self.steps:
            motor.setSpeed(power)
            if duration_s > 0:
                time.sleep(duration_s)


default_esc_initializer = SteppedInitializeESC(DEFAULT_ESC_STEPS)


class Motor():
    def __init__(
        self,
        pin,
        encoder: absoluteEncoder,
        pidController: Any = None,
        pidType: pidTypes = pidTypes.POSITION,
        esc_initializer: initializeESC | None = None,
    ):
        if pidController is None:
            pidController = PIDController(1, 0, 0)

        self.encoder = encoder
        self.motor = PWMOutputDevice(pin,frequency=50)

        self.pidController = pidController

        self.pidType = pidType
        self.esc_initializer = esc_initializer or default_esc_initializer

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
        self.esc_initializer.run(self)
    

    
motor = Motor
