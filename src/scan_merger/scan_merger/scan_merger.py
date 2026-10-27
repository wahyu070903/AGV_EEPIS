import os
import math
import yaml

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from ament_index_python.packages import get_package_share_directory


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')

        config_path = os.path.join(
            get_package_share_directory('scan_merger'),
            'config',
            'param.yaml'
        )

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        params = config['scan_merger']['ros__parameters']

        for name, value in params.items():
            self.declare_parameter(name, value)

        p = self.get_parameter
        self.target_frame = p('target_frame').value
        self.angle_min = p('angle_min').value
        self.angle_max = p('angle_max').value
        self.angle_inc = p('angle_increment').value
        self.range_min = p('range_min').value
        self.range_max = p('range_max').value
        self.rate = p('publish_rate').value
        self.max_age = p('max_scan_age').value
        self.exclude_box = list(p('exclude_box').value)
        self.exclude_radius = p('exclude_radius').value


        if len(self.exclude_box) != 4:
            self.get_logger().warn('exclude_box must contain 4 values [x_min, x_max, y_min, y_max], ignoring')
            self.exclude_box = [0.0, 0.0, 0.0, 0.0]
        self.n_out = int(round((self.angle_max - self.angle_min) / self.angle_inc)) + 1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scans = [None, None]          
        self.recv_time = [None, None]     

        self.create_subscription(
            LaserScan, p('scan_topic_1').value,
            lambda m: self._cb(0, m), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, p('scan_topic_2').value,
            lambda m: self._cb(1, m), qos_profile_sensor_data)

        self.pub = self.create_publisher(
            LaserScan, p('output_topic').value, qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            f'Merge {p("scan_topic_1").value} + {p("scan_topic_2").value} '
            f'-> {p("output_topic").value} (frame: {self.target_frame}, {self.n_out} points)')

    def _cb(self, i, msg):
        self.scans[i] = msg
        self.recv_time[i] = self.get_clock().now()

    def _lookup(self, scan):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, scan.header.frame_id,
                Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0.05))
        except TransformException:
            try: 
                return self.tf_buffer.lookup_transform(
                    self.target_frame, scan.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(
                    f'TF {scan.header.frame_id}->{self.target_frame} failed: {e}',
                    throttle_duration_sec=2.0)
                return None

    def _to_target_polar(self, scan):
        n = len(scan.ranges)
        r = np.asarray(scan.ranges, dtype=np.float64)
        if len(scan.intensities) == n:
            it = np.asarray(scan.intensities, dtype=np.float64)
        else:
            it = np.zeros(n)

        ang = scan.angle_min + np.arange(n) * scan.angle_increment
        valid = (np.isfinite(r)
                 & (r >= max(scan.range_min, self.range_min))
                 & (r <= min(scan.range_max, self.range_max)))
        if not np.any(valid):
            return None
        r, ang, it = r[valid], ang[valid], it[valid]

        tf = self._lookup(scan)
        if tf is None:
            return None

        pts = np.vstack((r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)))
        R = quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        pts = R @ pts + np.array([[t.x], [t.y], [t.z]])

        x, y = pts[0], pts[1]

        keep = np.ones(x.shape, dtype=bool)
        x0, x1, y0, y1 = self.exclude_box
        if (x1 > x0) and (y1 > y0):
            keep &= ~((x >= x0) & (x <= x1) & (y >= y0) & (y <= y1))
        if self.exclude_radius > 0.0:
            keep &= np.hypot(x, y) > self.exclude_radius
        if not np.any(keep):
            return None
        x, y, it = x[keep], y[keep], it[keep]

        out_r = np.hypot(x, y)
        out_a = np.arctan2(y, x)
        return out_a, out_r, it

    def _publish(self):
        now = self.get_clock().now()
        angles, ranges, ints, stamps = [], [], [], []

        for scan, rt in zip(self.scans, self.recv_time):
            if scan is None or (now - rt).nanoseconds * 1e-9 > self.max_age:
                continue
            res = self._to_target_polar(scan)
            if res is None:
                continue
            a, r, i = res
            angles.append(a)
            ranges.append(r)
            ints.append(i)
            stamps.append(Time.from_msg(scan.header.stamp))

        if not angles:
            return

        a = np.concatenate(angles)
        r = np.concatenate(ranges)
        it = np.concatenate(ints)

        idx = np.round((a - self.angle_min) / self.angle_inc).astype(int)
        ok = (idx >= 0) & (idx < self.n_out) & (r <= self.range_max) & (r >= self.range_min)
        idx, r, it = idx[ok], r[ok], it[ok]

        out_ranges = np.full(self.n_out, np.inf, dtype=np.float32)
        out_ints = np.zeros(self.n_out, dtype=np.float32)

        if idx.size:
            order = np.argsort(r)
            uniq, first = np.unique(idx[order], return_index=True)
            sel = order[first]
            out_ranges[uniq] = r[sel]
            out_ints[uniq] = it[sel]

        msg = LaserScan()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = max(stamps).to_msg()
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_min + (self.n_out - 1) * self.angle_inc)
        msg.angle_increment = float(self.angle_inc)
        msg.scan_time = 1.0 / self.rate
        msg.time_increment = 0.0
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = out_ranges.tolist()
        msg.intensities = out_ints.tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()