import can
import struct 

class CanBusRX():
    def __init__(self, bus):
        self.bus = bus
        self.address_table = {
            "ultrasonic_line1": 0x400,
            "ultrasonic_line2": 0x401,
        }

        self.dlc_length = {
            "ultrasonic_line1" : 8,
            "ultrasonic_line2" : 8,
        }

        self.ultrasonic_data = [None, None, None, None]

    def read(self):
        message = self.bus.recv(timeout=0.0)
        if message is None:
            return
        
        self.read_ultrasonic(message)

    def read_ultrasonic(self, message):

        if message.arbitration_id == self.address_table["ultrasonic_line1"] and message.dlc == self.dlc_length["ultrasonic_line1"]:
            ch1, ch2 = struct.unpack("<ff", message.data)
            self.ultrasonic_data[0] = ch1
            self.ultrasonic_data[1] = ch2

        if message.arbitration_id == self.address_table["ultrasonic_line2"] and message.dlc == self.dlc_length["ultrasonic_line2"]:
            ch3, ch4 = struct.unpack("<ff", message.data)
            self.ultrasonic_data[2] = ch3
            self.ultrasonic_data[3] = ch4

        for data in self.ultrasonic_data:
            if data is None:
                return 
        
        copy = self.ultrasonic_data
        self.ultrasonic_data = [None, None, None, None]

        return copy
