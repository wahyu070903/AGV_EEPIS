import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

MESSAGE_TABLE = {
    0x010: {
        "name" : "low_level_status",
        "dlc" : 4,
        "fmt" : "<i",
        "group" : "ll_status",
        "slice" : (0,1)
    },

    0x011: {
        "name": "right",
        "dlc": 4,
        "fmt": "<i",
        "group": "enc_ticks",
        "slice": (0, 1),
    },
    
    0x012: {
        "name": "left",
        "dlc": 4,
        "fmt": "<i",
        "group": "enc_ticks",
        "slice": (1, 2),
    },

    0x020: {
        "name": "ultrasonic_line1",
        "dlc": 8,
        "fmt": "<ff",
        "group": "ultrasonic",
        "slice": (0, 2),
    },

    0x021: {
        "name": "ultrasonic_line2",
        "dlc": 8,
        "fmt": "<ff",
        "group": "ultrasonic",
        "slice": (2, 4),
    },
}

GROUP_TOPICS = {
    "ultrasonic": "/ultrasonic",
    "enc_ticks": "/low_level/enc_ticks",
    "ll_status" : "/low_level/status",
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

        self.group_msg_types = {}
        for spec in MESSAGE_TABLE.values():
            group = spec["group"]
            fmt = spec["fmt"]

            msg_type = self._get_ros_message_type(fmt)

            if group in self.group_msg_types:
                if self.group_msg_types[group] != msg_type:
                    raise ValueError(
                        f"Group '{group}' has multiple ROS message types"
                    )
            else:
                self.group_msg_types[group] = msg_type

        self.group_publishers = {
            group: self.create_publisher(self.group_msg_types[group],GROUP_TOPICS[group],10)
            for group in self.group_buffers
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

    def _get_ros_message_type(self, fmt):
        if fmt == "<ff":
            return Float32MultiArray

        elif fmt == "<i":
            return Int32MultiArray

        else:
            raise ValueError(f"Unsupported CAN format: {fmt}")

    def _update_group(self, group, slice_range, values):
        start, end = slice_range

        buf = self.group_buffers[group]
        buf[start:end] = values

        if any(v is None for v in buf):
            return

        msg_type = self.group_msg_types[group]
        msg = msg_type()

        msg.data = list(buf)

        self.group_publishers[group].publish(msg)

        self.group_buffers[group] = [None] * len(buf)