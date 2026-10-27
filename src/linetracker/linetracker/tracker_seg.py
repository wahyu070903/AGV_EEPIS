import time
import cv2
import torch
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO


class Tracker_Seg(Node):
    def __init__(self):
        super().__init__('tracker_seg')

        # Parameters
        self.declare_parameter('model_path', '/home/orsted-793/AGV_EEPIS/src/linetracker/dataset/best_v3.pt')
        # TODO: camera id need to fix even after restart 
        self.declare_parameter('camera_id', 2)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('test_display', True)

        model_path = self.get_parameter('model_path').value
        camera_id = self.get_parameter('camera_id').value
        imgsz = self.get_parameter('imgsz').value
        conf = self.get_parameter('conf').value
        self.test_display = self.get_parameter('test_display').value

        self.imgsz = int(imgsz)
        self.conf = float(conf)

        # Check CUDA
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Inference device: {self.device}')

        if torch.cuda.is_available():
            self.get_logger().info(
                f'GPU: {torch.cuda.get_device_name(0)}'
            )

        # Load YOLO segmentation model
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)

        # Open USB camera
        self.cap = cv2.VideoCapture(
            int(camera_id), cv2.CAP_V4L2
        )

        if not self.cap.isOpened():
            raise RuntimeError(
                f'Cannot open camera ID {camera_id}'
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.sample_ratios = [0.875, 0.8125, 0.75, 0.6875, 0.625, 0.5625]

        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(
            Image, '/tracker_seg/image', 10
        )

        self.mask_count_pub = self.create_publisher(
            Float32MultiArray, '/tracker_seg/info', 10
        )
        
        self.tracker_pub = self.create_publisher(
            Float32MultiArray, '/tracker_seg', 10
        )

        self.frame_count = 0
        self.last_log_time = time.time()
        
        self.trajectory_available = False

        # Camera and inference loop: 20 Hz maximum
        self.timer = self.create_timer(
            0.05, self.process_frame
        )

        self.get_logger().info('Tracker segmentation node started.')

    def process_frame(self):
        ok, frame = self.cap.read()

        if not ok:
            self.get_logger().warning(
                'Failed to read camera frame'
            )
            return

        try:
            result = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                retina_masks=True,
                verbose=False
            )[0]

            # Draw masks, boxes and class labels
            annotated = result.plot()
            
            # Get mask data for trajectory calculate
            error_px, error_deg, points = None, None, None
            if result.masks is not None:
                mask_array = result.masks.data.cpu().numpy()
                mask_count = len(mask_array)

                if mask_count > 0:
                    first_mask = mask_array[0]
                    binary_mask_img = (first_mask * 255).astype(np.uint8)

                    error_px, error_deg, points = self.calculate_trajectory_error(binary_mask_img)

                    if error_deg is not None:
                        msg = Float32MultiArray()
                        msg.data = [1.0, float(error_px), float(error_deg)]
                        self.tracker_pub.publish(msg)

                    else:
                        msg = Float32MultiArray()
                        msg.data = [-1.0, 0.0, 0.0]
                        self.tracker_pub.publish(msg)

            if self.test_display:
                frame_copy = annotated.copy()
                height, width = frame_copy.shape[:2]

                for ratio in self.sample_ratios:
                    y = int(ratio * (height - 1))

                    cv2.line(
                        frame_copy,
                        (0, y),
                        (width - 1, y),
                        (255, 255, 0),
                        1
                    )

                height, width, _ = frame_copy.shape

                top_x = int(width / 2)
                top_y = int(height * 0.2)

                bottom_x = int(width / 2)
                bottom_y = height

                cv2.line(
                    frame_copy, 
                    (bottom_x, bottom_y), 
                    (top_x, top_y),        
                    (0, 255, 0),         
                    2                      
                )

                offset_bottom = int(width * 0.3) # Lebar bukaan di bawah
                offset_top = int(width * 0.05)
                
                cv2.line(frame_copy, (bottom_x - offset_bottom, bottom_y), (top_x - offset_top, top_y), (255, 0, 0), 2)
                cv2.line(frame_copy, (bottom_x + offset_bottom, bottom_y), (top_x + offset_top, top_y), (255, 0, 0), 2)

                if error_deg is not None:
                    cv2.putText(
                        frame_copy,
                        f'Points: {len(points)}',
                        (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame_copy,
                        f'error_angle: {error_deg:.1f} deg',
                        (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

                    cv2.putText(
                        frame_copy,
                        f'error_px: {error_px:.1f} px',
                        (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

                    cv2.circle(
                        frame_copy,
                        (int(points[-1][0]), int(points[-1][1])),
                        6,
                        (0, 255, 255),
                        -1
                    )

                else:
                    cv2.putText(
                        frame_copy,
                        f'no line detected',
                        (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )

                cv2.imshow('test_display', frame_copy)
                cv2.waitKey(1)

            # Publish annotated image
            image_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8'
            )

            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_link'
            self.image_pub.publish(image_msg)

            # Publish detection information:
            # [number_of_masks, inference_time_ms]
            info_msg = Float32MultiArray()
            mask_count = (
                0 if result.masks is None
                else len(result.masks.data)
            )
            inference_ms = result.speed.get('inference', 0.0)

            info_msg.data = [
                float(mask_count),
                float(inference_ms)
            ]
            self.mask_count_pub.publish(info_msg)

            self.frame_count += 1
            now = time.time()

            if now - self.last_log_time >= 2.0:
                fps = self.frame_count / (
                    now - self.last_log_time
                )
                self.get_logger().info(
                    f'FPS: {fps:.1f} | '
                    f'Masks: {mask_count} | '
                    f'Inference: {inference_ms:.1f} ms'
                )
                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(
                f'Inference error: {e}'
            )

    def calculate_trajectory_error(self, mask):
        height, width = mask.shape[:2]
        robot_x = width / 2.0
        
        sample_rows = [int(r * (height - 1)) for r in self.sample_ratios]
        points = []

        for y in sample_rows:
            xs = np.where(mask[y, :] > 0)[0]

            if len(xs) == 0:
                continue

            x_center = float(np.mean(xs))
            points.append((x_center, float(y))) 

        if len(points) < 2:
            return None, None, points

        lookahead_x, lookahead_y = points[0]
        lateral_error_px = lookahead_x - robot_x

        pts = np.asarray(points, dtype=np.float64)
        a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)

        heading_error_deg = np.degrees(np.arctan(a))
        return lateral_error_px, heading_error_deg, points

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Tracker_Seg()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Node error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
