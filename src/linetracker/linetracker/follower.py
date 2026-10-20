import rclpy
import numpy as np
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

DEG_TO_RAD = np.pi / 180.0

class Follower(Node):
    def __init__(self):
        super().__init__('follower')

        self.tracker_sub = self.create_subscription(
            Float32MultiArray,
            '/tracker_seg',
            self.tracker_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

        self.kp = 0.35
        self.ki = 0.0
        self.kd = 0.0

        self.base_speed = 0.14
        self.max_turn = 0.24

        self.feedback = 0
        self.is_line_exist = False
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        self.last_line_time = None
        self.line_timeout = 0.30

        self.get_logger().info('Camera line follower started')

    def tracker_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.publish_stop()
            return 
        
        if msg.data[0] == -1.0:
            self.is_line_exist = False
        elif msg.data[0] == 1.0:
            self.is_line_exist = True

        if self.is_line_exist is False:
            self.publish_stop()
            return

        self.feedback = float(msg.data[2])
        self.last_line_time = time.monotonic()

    def publish_stop(self):
        msg = Twist()
        self.cmd_publisher.publish(msg)

    def timer_callback(self):
        # now = time.monotonic()
        # # data kadaluarsa
        # if self.last_line_time is None or now - self.last_line_time > self.line_timeout:
        #     self.publish_stop()
        #     self.prev_error = 0
        #     self.integral = 0
        #     self.prev_time = 0

        #     return
        
        # target = 0
        # error = target - self.feedback

        # if self.prev_time is None:
        #     dt = 0.02 #samakan sesuai ccallback timer
        # else:
        #     dt = now - self.prev_time
        
        # dt = max(dt, 0.001)

        # p_element = self.kp * error
        # self.integral += error * dt
        # self.integral = max(-1.0, min(self.integral, 1.0))

        # i_element = self.ki * self.integral

        # d_elemant = self.kd * ((error - self.prev_error) / dt)

        # angular = p_element + i_element + d_elemant
        # angular = (max(-self.max_turn, min(angular, self.max_turn))) #ubah tanda minus jika robot salah puttar

        # turn_ratio = min(abs(angular) / self.max_turn, 1.0)
        # linear = self.base_speed * (1.0 - 0.5 * turn_ratio)

        if self.is_line_exist is False:
            self.publish_stop()
            return

        cmd = Twist()
        cmd.linear.x = float(self.base_speed)
        cmd.angular.z = float(self.feedback * DEG_TO_RAD * -1.0)

        self.cmd_publisher.publish(cmd)

        # self.prev_error = error
        # self.prev_time = now

        # self.get_logger().debug(
        #     f'error={error:.3f}, '
        #     f'P={p_element:.3f}, I={i_element:.3f}, D={d_elemant:.3f}, '
        #     f'linear={linear:.3f}, '
        #     f'angular={angular:.3f}'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Follower()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError as error:
        print(error)

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()