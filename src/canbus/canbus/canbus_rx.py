import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can
import struct 

class CanBusRX(Node):
    def __init__(self, bus):
        super().__init__('can_rx')

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
        self.ultrasonic_pub = self.create_publisher(
            Float32MultiArray,
            '/ultrasonic',
            10
        )

    def read(self):
        while True:
            message = self.bus.recv(timeout=0.0)
            if message is None:
                break
            self.read_ultrasonic(message)
            
        return None

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

        pub_data = Float32MultiArray()
        pub_data.data = copy
        self.ultrasonic_pub.publish(pub_data)
        return copy
