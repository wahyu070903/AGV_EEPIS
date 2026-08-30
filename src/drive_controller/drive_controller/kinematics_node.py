"""
This only use for simulation
for real robot kinematic is preocessed on low level controller

Parameters:
    wheelbase (float)            : distance between front and rear axle [m], default 0.3
    wheel_radius (float)         : rolling radius of the drive wheel [m], default 0.05
    max_steering_angle (float)   : saturation limit for delta [rad], default 0.6
    max_velocity (float)         : saturation limit for v [m/s], default 6.0
    steering_joint_name (string) : name of steering joint in /joint_states, default 'steering_joint'
    wheel_joint_name (string)    : primary wheel joint name in /joint_states, default 'l_wheel_joint'
    wheel_joint_name_fallback    : used if primary joint is missing, default 'r_wheel_joint'
    odom_frame_id (string)       : default "odom"
    base_frame_id (string)       : default "base_footprint"
    publish_tf (bool)            : default True
    publish_joint_state (bool)   : default False (republish steering as a JointState)
    joint_state_timeout (float)  : seconds after which v/delta are zeroed if no /joint_states, default 0.5
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # ---- Parameters ----
        self.declare_parameter('wheelbase', 1.965)
        self.declare_parameter('wheel_radius', 0.235)
        self.declare_parameter('max_steering_angle', 0.785398163)
        self.declare_parameter('max_velocity', 2.2)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_joint_state', False)
        self.declare_parameter('steering_joint_name', 'steering_joint')
        self.declare_parameter('wheel_joint_name', 'l_wheel_joint')
        self.declare_parameter('wheel_joint_name_fallback', 'r_wheel_joint')
        self.declare_parameter('joint_state_timeout', 0.5)

        self.L = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_delta = self.get_parameter('max_steering_angle').value
        self.max_v = self.get_parameter('max_velocity').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        self.publish_joint_state_flag = self.get_parameter('publish_joint_state').value
        self.steering_joint_name = self.get_parameter('steering_joint_name').value
        self.wheel_joint_name = self.get_parameter('wheel_joint_name').value
        self.wheel_joint_name_fallback = self.get_parameter('wheel_joint_name_fallback').value
        self.joint_state_timeout = self.get_parameter('joint_state_timeout').value

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Feedback measured from /joint_states (used for odometry)
        self.v_meas = 0.0
        self.delta_meas = 0.0
        self.last_joint_state_time = None   # rclpy Time from message header
        self.last_joint_state_wall_time = self.get_clock().now()  # for timeout/staleness check

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Pub/Sub ----
        self.cmd_sub = self.create_subscription(
            Twist, '/radio/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.wheel_vel_pub = self.create_publisher(Float64, '/robot_model/wheel_cmd_vel', 10)
        self.steering_pub = self.create_publisher(Float64, '/robot_model/steer_cmd_pos', 10)

        if self.publish_joint_state_flag:
            self.joint_pub = self.create_publisher(JointState, '/joint_states_echo', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Bicycle kinematics node started (odometry from /joint_states). '
            f'wheelbase={self.L} m, wheel_radius={self.wheel_radius} m, '
            f'max_v={self.max_v} m/s, max_delta={self.max_delta} rad')

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        delta = msg.angular.z
        command_max = 100
        v = (v / command_max) * self.max_v
        delta = (delta / command_max) * self.max_delta

        self.wheel_vel_pub.publish(Float64(data=v))
        self.steering_pub.publish(Float64(data=delta))

    def joint_state_callback(self, msg: JointState):
            steer_idx = msg.name.index(self.steering_joint_name)
            delta = msg.position[steer_idx] * -1.0
        except (ValueError, IndexError):
            delta = self.delta_meas 

        wheel_omega = None
        for jname in (self.wheel_joint_name, self.wheel_joint_name_fallback):
            try:
                widx = msg.name.index(jname)
                wheel_omega = msg.velocity[widx]
                break
            except (ValueError, IndexError):
                continue

        if wheel_omega is None:
            self.get_logger().warn(
                f'Neither "{self.wheel_joint_name}" nor "{self.wheel_joint_name_fallback}" '
                f'found in /joint_states; skipping this update.', throttle_duration_sec=5.0)
            return

        v = wheel_omega * self.wheel_radius * -1.0

        v = max(-self.max_v, min(self.max_v, v))
        delta = max(-self.max_delta, min(self.max_delta, delta))

        stamp = Time.from_msg(msg.header.stamp) if (
            msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
        ) else self.get_clock().now()

        if self.last_joint_state_time is not None:
            dt = (stamp - self.last_joint_state_time).nanoseconds * 1e-9
        else:
            dt = 0.0

        self.last_joint_state_time = stamp
        self.last_joint_state_wall_time = self.get_clock().now()

        self.v_meas = v
        self.delta_meas = delta

        if dt <= 0.0:
            # first message, or non-increasing stamp: just publish current pose, no integration
            self.publish_odom(stamp, self.v_meas, 0.0)
            if self.publish_tf_flag:
                self.publish_tf(stamp)
            return

        # ---- Bicycle kinematic integration (rear-axle model) ----
        theta_dot = self.v_meas * math.tan(self.delta_meas) / self.L
        self.x += self.v_meas * math.cos(self.theta) * dt
        self.y += self.v_meas * math.sin(self.theta) * dt
        self.theta += theta_dot * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # wrap

        self.publish_odom(stamp, self.v_meas, theta_dot)

        if self.publish_tf_flag:
            self.publish_tf(stamp)
        if self.publish_joint_state_flag:
            self.publish_joint_state(stamp, self.delta_meas)

    def publish_odom(self, stamp, v, theta_dot):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = theta_dot
        self.odom_pub.publish(odom)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_state(self, stamp, delta):
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = [self.steering_joint_name]
        js.position = [delta]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





