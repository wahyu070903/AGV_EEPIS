import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class KinematicsDDMR(Node):
    def __init__(self):
        super().__init__('kinematics_ddmr')

        self.encoder_ppr = 20
        self.gearRatio = 30.0
        self.wheel_diameter = 0.068      
        self.wheel_base = 0.19             
        self.ticks_per_revolution = self.encoder_ppr * self.gearRatio   
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meter_per_tick = self.wheel_circumference / self.ticks_per_revolution

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        
        self.enc_sub = self.create_subscription(
            Int32MultiArray, 
            '/low_level/enc_ticks',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(
                'Encoder data must contain [left, right]'
            )
            return

        right_ticks = msg.data[0]
        left_ticks = msg.data[1]

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_time = self.get_clock().now()
            return
        
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        delta_left = delta_left_ticks * self.meter_per_tick
        delta_right = delta_right_ticks * self.meter_per_tick
        delta_s = (delta_right + delta_left) / 2.0

        delta_theta = (delta_right - delta_left) / self.wheel_base

        theta_mid = self.theta + delta_theta / 2.0

        self.x += delta_s * math.cos(theta_mid)
        self.y += delta_s * math.sin(theta_mid)

        self.theta += delta_theta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        current_time = self.get_clock().now()

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        if dt <= 0.0:
            return

        linear_velocity = delta_s / dt
        angular_velocity = delta_theta / dt

        self.publish_odom(
            current_time,
            linear_velocity,
            angular_velocity
        )

    def publish_odom(self, current_time, linear_velocity, angular_velocity):
        q = Quaternion()

        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        odom = Odometry()

        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    999,  0,    0,    0,
            0,    0,    0,    999, 0,    0,
            0,    0,    0,    0,    999, 0,
            0,    0,    0,    0,    0,    0.1
        ]

        odom.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    999,  0,    0,    0,
            0,    0,    0,    999, 0,    0,
            0,    0,    0,    0,    999, 0,
            0,    0,    0,    0,    0,    0.1
        ]

        self.odom_pub.publish(odom)

        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = q

        self.tf_broadcaster.sendTransform(transform)

    def cmd_callback(self, msg: Twist):
        pass


def main(args=None):

    rclpy.init(args=args)

    node = KinematicsDDMR()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()