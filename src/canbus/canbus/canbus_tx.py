import struct
import can

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CanBusTX(Node):
    def __init__(self, bus):
        super().__init__('can_tx')
        self.bus = bus
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.send_cmd_msg,
            10
        )

    def send_cmd_msg(self, msg: Twist):
        linear = msg.linear.x
        angular =  msg.angular.z

        data = struct.pack(
            '<ff',
            linear,
            angular
        )

        can_msg = can.Message(
            arbitration_id=0x001,
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(can_msg)

            # self.get_logger().info(
            #     f'TX CAN: linear={linear:.2f}, angular={angular:.2f}'
            # )

        except can.CanError as e:
            self.get_logger().error(
                f'CAN send failed: {e}'
            )
