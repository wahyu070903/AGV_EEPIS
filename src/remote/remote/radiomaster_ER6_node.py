import serial
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class CRSFDecoder:

    RC_CHANNELS_PACKED = 0x16

    def __init__(self):
        self.buffer = bytearray()

    @staticmethod
    def to_percent(value):
        # -100 ... 0 ... +100
        calculation = (value - 992) / 819 * 100
        return int(calculation)   
        
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

            if total_length < 4 or total_length > 64:
                del self.buffer[0]
                continue

            if len(self.buffer) < total_length:
                break 

            frame = bytes(self.buffer[:total_length])

            crc_received = frame[-1]
            crc_calculated = self.crc8(frame[2:-1])

            if crc_received != crc_calculated:
                del self.buffer[0]
                continue

            del self.buffer[:total_length]

            frame_type = frame[2]

            if frame_type == self.RC_CHANNELS_PACKED:
                payload = frame[3:-1]
                channels = self.decode_channels(payload)

                if channels is not None:
                    frames.append(channels)

        return frames

    def decode_channels(self, payload):
        if len(payload) != 22:
            return None

        channels = []

        bit_buffer = 0
        bit_count = 0

        index = 0

        for _ in range(16):

            while bit_count < 11:

                bit_buffer |= (payload[index] << bit_count)

                bit_count += 8
                index += 1

            channel = bit_buffer & 0x7FF

            bit_buffer >>= 11
            bit_count -= 11
            topercent = self.to_percent(channel)
            channels.append(topercent)

        return channels


class RadioReceiver(Node):

    def __init__(self):

        super().__init__('radio_receiver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.port = self.get_parameter('port').value

        self.declare_parameter('baudrate', 420000)
        self.baudrate = self.get_parameter('baudrate').value
        
        self.declare_parameter('sim', False)
        self.sim = self.get_parameter('sim').value
        self.lastNoData = None
        self.warnIsprinted = False

        try:

            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'\033[91m[Remote] Failed to open serial port: {e}\033[0m'
            )

            raise

        self.decoder = CRSFDecoder()

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/radio/channels',
            10
        )

        self.simPublisher = self.create_publisher(
            Twist,
            '/radio/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.005,
            self.read_serial
        )

        self.get_logger().info(
            f'\033[92m[Remote] CRSF receiver started: '
            f'{self.port} @ {self.baudrate}\033[0m'
        )

    def read_serial(self):
        try:
            data = self.serial.read(self.serial.in_waiting)

            if not data:
                now = time.time()
                if self.lastNoData is None:
                    self.lastNoData = now

                elif now - self.lastNoData > 5.0:
                    if not self.warnIsprinted:
                        self.get_logger().warn(
                            '\033[93m[Remote] No data received. '
                            'Check remote is connected?\033[0m'
                        )
                        self.warnIsprinted = True
                return
            else:
                self.lastNoData = None
                self.warnIsprinted = False
                if self.warnIsprinted:
                    self.get_logger().info(
                        '\033[94m[Remote] Data Received'
                    )

            frames = self.decoder.feed(data)
            for channels in frames:

                msg = Float32MultiArray()

                msg.data = [
                    float(channel)
                    for channel in channels
                ]

                self.publisher.publish(msg)

                if self.sim is True:
                    msg_twist = Twist()
                    throttle = channels[1]
                    steering = channels[3]
                    msg_twist.linear.x = float(throttle)
                    msg_twist.linear.y = 0.0
                    msg_twist.linear.z = 0.0

                    msg_twist.angular.x = 0.0
                    msg_twist.angular.y = 0.0
                    msg_twist.angular.z = float(steering)

                    self.simPublisher.publish(msg_twist)

        except serial.SerialException as e:

            self.get_logger().error(
                f'\033[91m[Remote] Serial error: {e}\033[0m'
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