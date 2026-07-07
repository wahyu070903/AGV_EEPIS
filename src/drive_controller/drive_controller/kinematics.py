import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SteerDriveKinematics(Node):

    def __init__(self):
        super().__init__('steer_drive_kinematics')

        self.declare_parameter('wheel_radius', 0.10)               
        self.declare_parameter('front_axle_x', -0.902)             
        self.declare_parameter('rear_axle_x', 1.14)                
        self.declare_parameter('max_steering_angle', 0.785398)     
        self.declare_parameter('steering_joint_name', 'steering_joint')
        self.declare_parameter('left_wheel_joint_name', 'l_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'r_wheel_joint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.x_f = float(self.get_parameter('front_axle_x').value)
        self.x_r = float(self.get_parameter('rear_axle_x').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.steer_joint = self.get_parameter('steering_joint_name').value
        self.l_wheel_joint = self.get_parameter('left_wheel_joint_name').value
        self.r_wheel_joint = self.get_parameter('right_wheel_joint_name').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.cmd_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        self.L = self.x_r - self.x_f  # effective wheelbas
        if abs(self.L) < 1e-6:
            self.get_logger().error(
                'Wheelbase (L) terhitung ~0. Periksa parameter front_axle_x / rear_axle_x!')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = None
        self.last_cmd_time = self.get_clock().now()

        # ---------------- Pub / Sub ----------------
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 50)

        self.steer_cmd_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.02, self.cmd_timeout_check)  # 50 Hz watchdog

        self.get_logger().info(
            f'Steer-drive kinematics aktif. Wheelbase L={self.L:.3f} m, '
            f'wheel_radius={self.wheel_radius:.3f} m (PASTIKAN NILAI INI BENAR!)')

    # =======================================================================
    # INVERSE KINEMATICS: cmd_vel -> perintah steering + roda
    # =======================================================================
    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx_cmd = msg.linear.x
        wz_cmd = msg.angular.z

        if abs(vx_cmd) < 1e-4 and abs(wz_cmd) < 1e-4:
            delta = 0.0
            v = 0.0
        else:
            delta = math.atan2(wz_cmd * self.L, vx_cmd)
            delta_clamped = max(-self.max_steer, min(self.max_steer, delta))
            if abs(delta_clamped - delta) > 1e-6:
                self.get_logger().warn(
                    f'Sudut steering diminta {math.degrees(delta):.1f} deg melebihi limit '
                    f'{math.degrees(self.max_steer):.1f} deg, dipotong (clamped).',
                    throttle_duration_sec=2.0)
            delta = delta_clamped

            cos_d = math.cos(delta)
            if abs(cos_d) < 1e-3:
                # delta mendekati 90 derajat (di luar limit fisik robot ini, jaga-jaga saja)
                v = (wz_cmd * self.L / math.sin(delta)) if abs(math.sin(delta)) > 1e-6 else 0.0
            else:
                v = vx_cmd / cos_d

        wheel_ang_vel = v / self.wheel_radius if self.wheel_radius > 1e-6 else 0.0

        steer_msg = Float64MultiArray()
        steer_msg.data = [delta]
        self.steer_cmd_pub.publish(steer_msg)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_ang_vel, wheel_ang_vel]  # l_wheel, r_wheel disamakan
        self.wheel_cmd_pub.publish(wheel_msg)

    def cmd_timeout_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0]
            self.wheel_cmd_pub.publish(stop_msg)  # steering tetap di posisi terakhir

    # =======================================================================
    # FORWARD KINEMATICS: joint_states -> odometry
    # =======================================================================
    def joint_state_cb(self, msg: JointState):
        try:
            idx_steer = msg.name.index(self.steer_joint)
            idx_l = msg.name.index(self.l_wheel_joint)
            idx_r = msg.name.index(self.r_wheel_joint)
        except ValueError:
            return  # joint belum terpublish

        if len(msg.position) <= idx_steer or len(msg.velocity) <= max(idx_l, idx_r):
            return

        delta = msg.position[idx_steer]
        w_l = msg.velocity[idx_l]
        w_r = msg.velocity[idx_r]

        v = self.wheel_radius * (w_l + w_r) / 2.0
        omega = (v * math.sin(delta) / self.L) if abs(self.L) > 1e-6 else 0.0
        vx_base = v * math.cos(delta)
        vy_base = -omega * self.x_r

        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            now = Time.from_msg(msg.header.stamp)
        else:
            now = self.get_clock().now()

        if self.last_odom_time is None:
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        delta_x = (vx_base * math.cos(self.theta) - vy_base * math.sin(self.theta)) * dt
        delta_y = (vx_base * math.sin(self.theta) + vy_base * math.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = math.atan2(
            math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

        self.publish_odom(now, vx_base, vy_base, omega)

    def publish_odom(self, stamp, vx, vy, omega):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SteerDriveKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()