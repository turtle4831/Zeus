from RobotSide.Subsystems.multiplexer import Multiplexer
from RobotSide.Utils.baseEncoder import baseEncoder


class absoluteEncoder(baseEncoder):
    def __init__(self, channel,multiplexer:Multiplexer):

        super().__init__(channel, multiplexer)
        try:
            self.multiplexer.add_device(channel)
        except ValueError as e:
            print(f"Error adding device to multiplexer, channel {channel}: {e}")
        
    def getPosition(self):
        return self.multiplexer.getEncoderData(self.channel)
    