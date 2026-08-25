import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class CRSFDecoder:

    RC_CHANNELS_PACKED = 0x16

    def __init__(self):
        self.buffer = bytearray()

    @staticmethod
    def crc8(data):
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF

        return crc

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:

            if len(self.buffer) < 4:
                break

            frame_length = self.buffer[1]

            total_length = frame_length + 2

            if len(self.buffer) < total_length:
                break

            frame = bytes(
                self.buffer[:total_length]
            )

            del self.buffer[:total_length]

            crc_received = frame[-1]

            crc_calculated = self.crc8(
                frame[2:-1]
            )

            if crc_received != crc_calculated:
                continue

            frame_type = frame[2]

            if frame_type == self.RC_CHANNELS_PACKED:

                channels = self.decode_channels(
                    frame[3:-1]
                )

                if channels is not None:
                    frames.append(channels)

        return frames

    @staticmethod
    def decode_channels(payload):

        if len(payload) != 22:
            return None

        channels = []

        bit_buffer = 0
        bit_count = 0

        index = 0

        for _ in range(16):

            while bit_count < 11:

                bit_buffer |= (
                    payload[index]
                    << bit_count
                )

                bit_count += 8
                index += 1

            channel = bit_buffer & 0x7FF

            bit_buffer >>= 11
            bit_count -= 11

            channels.append(channel)

        return channels


class RadioReceiver(Node):

    def __init__(self):

        super().__init__(
            'radio_receiver'
        )

        self.declare_parameter(
            'port',
            '/dev/ttyUSB0'
        )

        self.declare_parameter(
            'baudrate',
            420000
        )

        self.port = self.get_parameter(
            'port'
        ).value

        self.baudrate = self.get_parameter(
            'baudrate'
        ).value

        try:

            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Failed to open serial port: {e}'
            )

            raise

        self.decoder = CRSFDecoder()

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/radio/channels',
            10
        )

        self.timer = self.create_timer(
            0.005,
            self.read_serial
        )

        self.get_logger().info(
            f'CRSF receiver started: '
            f'{self.port} @ {self.baudrate}'
        )

    def read_serial(self):

        try:

            data = self.serial.read(
                self.serial.in_waiting
            )

            if not data:
                return

            frames = self.decoder.feed(data)

            for channels in frames:

                msg = Float32MultiArray()

                msg.data = [
                    float(channel)
                    for channel in channels
                ]

                self.publisher.publish(msg)

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = RadioReceiver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node.serial.is_open:
            node.serial.close()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()