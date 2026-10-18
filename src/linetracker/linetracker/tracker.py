import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        self.bridge = CvBridge()

        # EMA filter untuk angle
        self.ema_alpha = 0.2
        self.filtered_angle = None
        self.deadband = 1.2

        self.image_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10
        )

        self.track_publisher = self.create_publisher(
            Float32MultiArray,
            '/tracker',
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except CvBridgeError as e:
            self.get_logger().error(
                f'Image conversion failed: {e}'
            )
            return

        
        angle, error, binary, overlay = self.detect_line(frame)

        if angle is not None:
            # EMA filter
            if self.filtered_angle is None:
                self.filtered_angle = angle
            else:
                self.filtered_angle = (
                    self.ema_alpha * angle
                    + (1.0 - self.ema_alpha) * self.filtered_angle
                )

            angle_ema = self.filtered_angle
            if angle_ema < self.deadband and angle_ema > 0:
                angle_ema = 0
            elif angle_ema > -self.deadband and angle_ema < 0:
                angle_ema = 0 

            # Tampilkan angle mentah dan hasil filter
            cv2.putText(
                overlay,
                f'Raw Angle: {angle:.1f} deg',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2
            )

            cv2.putText(
                overlay,
                f'EMA Angle: {angle_ema:.1f} deg',
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            msg = Float32MultiArray()
            msg.data = [1.0, float(angle_ema)]
            self.track_publisher.publish(msg)

        else:
            cv2.putText(
                overlay,
                'LINE NOT FOUND',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            msg = Float32MultiArray()
            msg.data = [-1.0, 0.0]
            self.track_publisher.publish(msg)

        cv2.imshow('Line Detection', overlay)
        cv2.imshow('Binary ROI', binary)
        cv2.waitKey(1)


    def detect_line(self, frame):
        height, width = frame.shape[:2]

        # ROI: 40% bagian bawah gambar
        # roi_y = int(height * 0.60)
        roi_y = 0
        roi = frame[roi_y:height, :]

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # lowwer_yellow = np.array([20,100,100])
        # upper_yellow = np.array([35,255,255])

        lowwer_yellow = np.array([18,55,140])
        upper_yellow = np.array([35,255,255])

        binary = cv2.inRange(
            hsv,
            lowwer_yellow,
            upper_yellow
        )

        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        overlay = frame.copy()

        # Garis tengah kamera
        camera_cx = width // 2

        cv2.line(
            overlay,
            (camera_cx, roi_y),
            (camera_cx, height - 1),
            (255, 0, 0),
            2
        )

        if not contours:
            return None, None, binary, overlay

        contour = max(
            contours,
            key=cv2.contourArea
        )

        if cv2.contourArea(contour) < 100:
            return None, None, binary, overlay

        moments = cv2.moments(contour)

        if moments['m00'] == 0:
            return None, None, binary, overlay

        # Titik pusat kontur, koordinat ROI
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        error = cx - camera_cx

        # Konversi centroid ke koordinat gambar asli
        center = (cx, cy + roi_y)

        # Gambar kontur yang terdeteksi dengan warna hijau
        contour_full = contour.copy()
        contour_full[:, 0, 1] += roi_y

        cv2.drawContours(
            overlay,
            [contour_full],
            -1,
            (0, 255, 0),
            2
        )

        # Tandai titik pusat garis
        cv2.circle(
            overlay,
            center,
            6,
            (0, 255, 255),
            -1
        )

        # Estimasi orientasi garis menggunakan fitLine
        vx, vy, x0, y0 = cv2.fitLine(
            contour,
            cv2.DIST_L2,
            0,
            0.01,
            0.01
        ).flatten()

        # Arah panah: menuju bagian atas gambar
        # (arah maju pada perspektif kamera yang umum)
        if vy > 0:
            vx = -vx
            vy = -vy

        arrow_length = 100

        end_point = (
            int(center[0] + vx * arrow_length),
            int(center[1] + vy * arrow_length)
        )

        cv2.arrowedLine(
            overlay,
            center,
            end_point,
            (0, 0, 255),
            4,
            tipLength=0.25
        )

        # Tampilkan arah relatif garis
        angle = np.degrees(np.arctan2(vx, -vy))

        cv2.putText(
            overlay,
            f'Angle: {angle:.1f} deg',
            (20, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        return angle, error, binary, overlay

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Tracker()
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