from RobotSide.Subsystems.multiplexer import Multiplexer


class baseEncoder:
    def __init__(self, channel, multiplexer:Multiplexer):
        
        self.channel = channel
        self.multiplexer = multiplexer
        self.position = 0

    def getPosition(self):
        pass
    def getVelocity(self):
        pass
