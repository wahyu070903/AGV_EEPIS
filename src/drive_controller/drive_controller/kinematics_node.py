#!/usr/bin/env python3
"""
Bicycle kinematic model node for ROS2.

Subscribes:
    /cmd_vel (geometry_msgs/Twist)
        linear.x  -> forward velocity v [m/s]
        angular.z -> steering angle delta [rad]  (NOT yaw rate)

Publishes:
    /odom (nav_msgs/Odometry)
    TF: odom -> base_link
    /joint_states (optional, for the steering joint) if publish_joint_state:=true

Kinematic model (rear-axle reference point):
    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = v * tan(delta) / L

Parameters:
    wheelbase (float)            : distance between front and rear axle [m], default 0.3
    max_steering_angle (float)   : saturation limit for delta [rad], default 0.6
    max_velocity (float)         : saturation limit for v [m/s], default 2.0
    publish_rate (float)         : integration/publish rate [Hz], default 50.0
    odom_frame_id (string)       : default "odom"
    base_frame_id (string)       : default "base_link"
    publish_tf (bool)            : default True
    cmd_timeout (float)          : seconds after which cmd is zeroed if no new msg, default 0.5
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # ---- Parameters ----
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_joint_state', False)
        self.declare_parameter('steering_joint_name', 'front_steer_joint')
        self.declare_parameter('cmd_timeout', 0.5)

        self.L = self.get_parameter('wheelbase').value
        self.max_delta = self.get_parameter('max_steering_angle').value
        self.max_v = self.get_parameter('max_velocity').value
        rate = self.get_parameter('publish_rate').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        self.publish_joint_state_flag = self.get_parameter('publish_joint_state').value
        self.steering_joint_name = self.get_parameter('steering_joint_name').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---- Pub/Sub ----
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        if self.publish_joint_state_flag:
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.update)

        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'Bicycle kinematics node started. wheelbase={self.L} m, '
            f'max_v={self.max_v} m/s, max_delta={self.max_delta} rad')

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        delta = msg.angular.z

        # Saturate
        v = max(-self.max_v, min(self.max_v, v))
        delta = max(-self.max_delta, min(self.max_delta, delta))

        self.v_cmd = v
        self.delta_cmd = delta
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # Zero command if stale (safety)
        elapsed_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        v = self.v_cmd
        delta = self.delta_cmd
        if elapsed_since_cmd > self.cmd_timeout:
            v = 0.0
            delta = 0.0

        # ---- Bicycle kinematic integration (rear-axle model) ----
        theta_dot = v * math.tan(delta) / self.L
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += theta_dot * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # wrap

        self.publish_odom(now, v, theta_dot)
        if self.publish_tf_flag:
            self.publish_tf(now)
        if self.publish_joint_state_flag:
            self.publish_joint_state(now, delta)

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
