import rclpy
from rclpy.node import Node
import can
from .canbus_rx import CanBusRX

class CanBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        self.bus = can.Bus(
            interface="socketcan",
            channel="can0",
            bitrate=500000
        )

        self.canbus_rx = CanBusRX(self.bus)
        self.timer = self.create_timer(
            0.02,
            self.read_can
        )
    
    def read_can(self):
        self.canbus_rx.read()

def main(args=None):

    rclpy.init(args=args)

    node = CanBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()