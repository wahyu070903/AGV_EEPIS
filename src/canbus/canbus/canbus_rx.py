import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

MESSAGE_TABLE = {
    0x400: {
        "name": "ultrasonic_line1",
        "dlc": 8,
        "fmt": "<ff",
        "group": "ultrasonic",
        "slice": (0, 2),
    },

    0x401: {
        "name": "ultrasonic_line2",
        "dlc": 8,
        "fmt": "<ff",
        "group": "ultrasonic",
        "slice": (2, 4),
    },

    0x100: {
        "name": "right",
        "dlc": 4,
        "fmt": "<i",
        "group": "enc_ticks",
        "slice": (0, 1),
    },
    
    0x101: {
        "name": "left",
        "dlc": 4,
        "fmt": "<i",
        "group": "enc_ticks",
        "slice": (1, 2),
    }
}

GROUP_TOPICS = {
    "ultrasonic": "/ultrasonic",
    "enc_ticks": "/enc_ticks",
}


class CanBusRX(Node):
    def __init__(self, bus):
        super().__init__('can_rx')
        self.bus = bus

        group_sizes = {}
        for spec in MESSAGE_TABLE.values():
            group = spec["group"]
            _, end = spec["slice"]
            group_sizes[group] = max(group_sizes.get(group, 0), end)

        self.group_buffers = {
            group: [None] * size for group, size in group_sizes.items()
        }

        self.group_publishers = {
            group: self.create_publisher(Float32MultiArray, topic, 10)
            for group, topic in GROUP_TOPICS.items()
        }

    def read(self):
        while True:
            message = self.bus.recv(timeout=0.0)
            if message is None:
                break
            self.handle_message(message)

    def handle_message(self, message):
        spec = MESSAGE_TABLE.get(message.arbitration_id)
        if spec is None:
            # Skip unknown arbitration_id 
            return

        if message.dlc != spec["dlc"]:
            self.get_logger().warn(
                f"DLC mismatch for {spec['name']}: "
                f"expected {spec['dlc']}, got {message.dlc}"
            )
            return

        try:
            values = struct.unpack(spec["fmt"], message.data)
        except struct.error as e:
            self.get_logger().error(f"Failed to unpack {spec['name']}: {e}")
            return

        self._update_group(spec["group"], spec["slice"], values)

    def _update_group(self, group, slice_range, values):
        start, end = slice_range
        buf = self.group_buffers[group]
        buf[start:end] = values

        if any(v is None for v in buf):
            return  

        pub_data = Float32MultiArray()
        pub_data.data = list(buf)
        self.group_publishers[group].publish(pub_data)

        self.group_buffers[group] = [None] * len(buf)