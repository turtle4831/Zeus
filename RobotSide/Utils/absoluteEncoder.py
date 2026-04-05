from RobotSide.Subsystems.multiplexer import Multiplexer


class absoluteEncoder():
    def __init__(self, channel,multiplexer:Multiplexer):
        
        self.channel = channel
        self.multiplexer = multiplexer
        self.position = 0
        self.rotations = 0

        try:
            self.multiplexer.add_device(channel)
            self.initial_position = self.getPosition()
        except ValueError as e:
            print(f"Error adding device to multiplexer, channel {channel}: {e}")

        
    def getPosition(self):
        return self.multiplexer.getEncoderData(self.channel)
    
    def updatePosition(self):
        self.position = self.getPosition()
        if abs(self.position - self.previous_position) > 360:  # If the position has gone over 360 degrees, increment rotations
            self.rotations += 1
        self.previous_position = self.position

    def getTotalPosition(self):
        total_position = self.rotations * 4096 + self.position - self.initial_position
        return total_position