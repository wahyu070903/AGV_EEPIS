
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class Follower_Seg(Node):
    def __init__(self):
        super().__init__('follower_seg')

        # =========================
        # ROS 2 interfaces
        # =========================
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

        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        # =========================
        # Controller parameters
        # =========================

        # Initial gains; tune experimentally
        self.k_y = 0.8       # Lateral gain
        self.k_psi = 0.8     # Heading gain

        self.base_speed = 0.14  # m/s
        self.min_speed = 0.05   # m/s
        self.max_turn = 0.24    # rad/s

        # Reduce speed when angular correction is large
        self.speed_reduction = 0.5

        # Line data timeout
        self.line_timeout = 0.30  # seconds

        # =========================
        # Tracker state
        # =========================
        self.is_line_exist = False

        self.lateral_error = 0.0  # meters
        self.heading_error = 0.0  # degrees

        self.last_line_time = None

        self.get_logger().info(
            'Line follower started: lateral + heading control'
        )

    def tracker_callback(self, msg: Float32MultiArray):
        # Expected format:
        # [status, lateral_error_m, heading_error_deg]
        #
        # status =  1 : line detected
        # status = -1 : line lost

        if len(msg.data) < 3:
            self.is_line_exist = False
            self.publish_stop()
            self.get_logger().warning(
                'Invalid /tracker_seg message: expected 3 values'
            )
            return

        status = float(msg.data[0])

        if status == -1.0:
            self.is_line_exist = False
            self.last_line_time = None
            self.publish_stop()
            return

        if status != 1.0:
            self.is_line_exist = False
            self.publish_stop()
            return

        lateral = float(msg.data[1])
        heading_deg = float(msg.data[2])

        # Reject invalid measurements
        if not (
            math.isfinite(lateral)
            and math.isfinite(heading_deg)
        ):
            self.is_line_exist = False
            self.publish_stop()
            return

        self.lateral_error = lateral
        self.heading_error = heading_deg

        self.is_line_exist = True
        self.last_line_time = time.monotonic()

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_publisher.publish(msg)

    def timer_callback(self):
        now = time.monotonic()

        # Stop when the line is missing or data is stale
        if (
            not self.is_line_exist
            or self.last_line_time is None
            or now - self.last_line_time > self.line_timeout
        ):
            self.is_line_exist = False
            self.publish_stop()
            return

        # =========================
        # 1. Convert heading to radians
        # =========================
        e_y = self.lateral_error
        e_psi = math.radians(self.heading_error)

        # =========================
        # 2. Combined control
        # =========================
        # Positive error should produce a left turn.
        omega = (
            self.k_y * e_y
            + self.k_psi * e_psi
        )

        # Limit angular velocity
        omega = max(
            -self.max_turn,
            min(omega, self.max_turn)
        )

        # =========================
        # 3. Adaptive linear velocity
        # =========================
        turn_ratio = min(
            abs(omega) / self.max_turn,
            1.0
        )

        linear = self.base_speed * (
            1.0 - self.speed_reduction * turn_ratio
        )

        linear = max(self.min_speed, linear)

        # =========================
        # 4. Publish velocity command
        # =========================
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(omega)

        self.cmd_publisher.publish(cmd)

        self.get_logger().debug(
            f'lateral={e_y:.3f} m, '
            f'heading={self.heading_error:.2f} deg, '
            f'linear={linear:.3f} m/s, '
            f'angular={omega:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Follower()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
