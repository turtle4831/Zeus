from RobotSide.Subsystems.multiplexer import Multiplexer


class encoder:
    def __init__(self, channel,multiplexer:Multiplexer):
        self.channel = channel
        self.position = 0
        self.multiplexer = multiplexer
        try:
            self.multiplexer.add_device(channel)
        except ValueError as e:
            print(f"Error adding device to multiplexer, channel {channel}: {e}")
        
    def getPosition(self):
        return self.multiplexer.getEncoderData(self.channel)
    