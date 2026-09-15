import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import can

from .canbus_rx import CanBusRX
from .canbus_tx import CanBusTX


class CanBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        self.bus = can.Bus(
            interface="socketcan",
            channel="can0",
            bitrate=500000
        )

        self.canbus_rx = CanBusRX(self.bus)
        self.canbus_tx = CanBusTX(self.bus)

        self.timer = self.create_timer(
            0.02,
            self.read_can
        )

    def read_can(self):
        self.canbus_rx.read()


def main(args=None):
    rclpy.init(args=args)

    bridge = CanBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(bridge.canbus_rx)
    executor.add_node(bridge.canbus_tx)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        bridge.bus.shutdown()

        bridge.canbus_rx.destroy_node()
        bridge.canbus_tx.destroy_node()
        bridge.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()