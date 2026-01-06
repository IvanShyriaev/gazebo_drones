import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32  # Для публікації детекцій

import torch
import torch.nn as nn
import time
import cv2
import os

from ultralytics import YOLO

class CNNDetectorNode(Node):
    def __init__(self, namespace=''):
        super().__init__('yolo_detector')

        self.ns = f'/{namespace}' if namespace else ''

        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_ALL,
            depth = 1
        )

        self.sub = self.create_subscription(
            Image,
            f'{self.ns}/camera/image_raw',
            self.image_cb,
            qos
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            f'{self.ns}/target/pose',
            10
        )

        self.detection_pub = self.create_publisher(
            Int32,
            'yolo_detection',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            'camera/yolo_images',
            10
        )

        self.bridge = CvBridge()

        self.get_logger().info('Loading YOLO model...')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.model = YOLO('/home/remote_gazebo/ros2_ws/src/typhoon_camera/typhoon_camera/best.pt')
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info(f'YOLO loaded on {self.device}')
        self.get_logger().info(f'YOLO Detector node namespace: {self.ns if self.ns else "none"}')

        self.busy = False
        self.last_time = time.time()
        self.img_counter = 0
        self.detection_counter = 0  # Лічильник детекцій

        self.output_dir = './yolo_outputs'
        os.makedirs(self.output_dir, exist_ok=True)

    def image_cb(self, msg: Image):

        if self.busy:
            return

        self.busy = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLOv8 inference
            results = self.model(frame, imgsz=640, conf=0.6, verbose=False)
            boxes = results[0].boxes

            detected = boxes is not None and len(boxes) > 0

            if detected:
                # Публікуємо координати для дрона-камікадзе
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'

                pose_msg.pose.position.x = 10.0
                pose_msg.pose.position.y = 10.0
                pose_msg.pose.position.z = 2.0
                pose_msg.pose.orientation.w = 1.0

                self.target_pub.publish(pose_msg)

                # Публікуємо повідомлення про детекцію для мапи
                detection_msg = Int32()
                detection_msg.data = self.detection_counter
                self.detection_pub.publish(detection_msg)

                self.get_logger().info(
                    f'Target detected #{self.detection_counter}'
                )

                self.detection_counter += 1

            # Малюємо bounding boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    label = f'{self.model.names[cls]} {conf:.2f}'

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

            # Публікація image з детекціями
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            self.image_pub.publish(img_msg)

            # FPS лог
            now = time.time()
            fps = 1.0 / (now - self.last_time)
            self.last_time = now

            det_count = 0 if boxes is None else len(boxes)
            self.get_logger().info(f'Detections: {det_count} | FPS: {fps:.1f}')

            # (опційно) збереження на диск
            img_filename = os.path.join(
                self.output_dir,
                f'yolo_{self.img_counter:05d}.jpg'
            )
            cv2.imwrite(img_filename, frame)
            self.img_counter += 1

        except Exception as e:
            self.get_logger().error(str(e))

        finally:
            self.busy = False




def main():
    rclpy.init()
    
    import sys
    namespace = sys.argv[1] if len(sys.argv) > 1 else ''
    
    node = CNNDetectorNode(namespace=namespace)
    rclpy.spin(node)

if __name__ == '__main__':
    main()