try:
    from gpiozero import MCP3008
except ModuleNotFoundError:
    class MCP3008:
        def __init__(self, *args, **kwargs):
            raise ModuleNotFoundError("No module named 'gpiozero'")


class BeamBreak:
    def __init__(
        self,
        channel: int,
        threshold: float = 0.5,
        interruptedBelowThreshold: bool = True,
        analogSensor=None,
    ):
        if threshold < 0 or threshold > 1:
            raise ValueError("threshold must be between 0 and 1")

        self.threshold = threshold
        self.interruptedBelowThreshold = interruptedBelowThreshold
        self.sensor = analogSensor if analogSensor is not None else MCP3008(channel=channel)
        self.interrupted = False
        self.update()

    def getValue(self) -> float:
        return self.sensor.value

    def isInterrupted(self) -> bool:
        self.update()
        return self.interrupted

    def update(self) -> None:
        value = self.getValue()
        if self.interruptedBelowThreshold:
            self.interrupted = value < self.threshold
        else:
            self.interrupted = value > self.threshold
