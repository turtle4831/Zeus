#smbus will always error, it is installed on the robot
import smbus # type: ignore

#each multiplexer has 8 channels

class Multiplexer:
    def __init__(self, address):
        self.address = address
        self.bus = smbus.SMBus(1)
        self.devices = [] 

        for i in range(8):
            self.devices.append(0)
        

    def select_channel(self, channel):
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")

        self.bus.write_byte(self.address, 1 << channel)

    def add_device(self, channel):
        self.devices[channel] = 1

    def scanDevices(self):
        scanned_devices = []
        for i in range(128):
            try:
                self.bus.read_byte(i)
                scanned_devices.append(i)
            except:
                pass
        return scanned_devices
    
    def getAllData(self):
        data = []
        for channel in range(8):
            if self.devices[channel] == 1:
                self.select_channel(channel)
                data.append(self.read_encoder_data())
            else:
                data.append(None)
        return data
    
    def getEncoderData(self, channel):
        data = 0
        if self.devices[channel] == 1:
            self.select_channel(channel)
            data = self.read_encoder_data()
        else:
            raise ValueError(f"No device on channel {channel}")
        
        return data
    
    def read_encoder_data(self):

        ANGLE_REG = 0x0E

        high_byte = self.bus.read_byte_data(self.address, ANGLE_REG)
        low_byte = self.bus.read_byte_data(self.address, ANGLE_REG + 1)

        angle = (high_byte << 8) | low_byte

        scaled_angle = angle * (360.0 / 4096.0)

        return scaled_angle

