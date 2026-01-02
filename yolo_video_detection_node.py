import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

import torch
import torch.nn as nn
import time
import cv2
import os

class CNNDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_ALL,
            depth = 1
        )

        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_cb,
            qos
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            'target/pose',
            10
        )

        self.bridge = CvBridge()

        self.get_logger().info('Loading YOLO model...')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'yolov5s',
            pretrained=True
        )
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info(f'YOLO loaded on {self.device}')

        self.busy = False
        self.last_time = time.time()
        self.img_counter = 0

        self.output_dir = './yolo_outputs'
        self.raw_dir = './raw_images'
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.raw_dir, exist_ok=True)


    def image_cb(self, msg: Image):

        if self.busy:
            return

        self.busy = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            raw_img_filename = os.path.join(self.raw_dir, f'raw_img_{self.img_counter:05d}.jpg')
            cv2.imwrite(raw_img_filename, frame)

            results = self.model(frame, size = 640)

            detections = results.xyxy[0]  

            # if len(detections) > 0:
            #     # time.sleep(5000)
            #     msg = PoseStamped()
            #     msg.header.stamp = self.get_clock().now().to_msg()
            #     msg.header.frame_id = 'map'

            #     #Тут просто заадємо координати const (поки що потім буде йоло з обчисленням координат)
            #     msg.pose.position.x = 10.0
            #     msg.pose.position.y = 10.0
            #     msg.pose.position.z = 2.0

            #     msg.pose.orientation.w = 1.0

            #     self.target_pub.publish(msg)

            #     self.get_logger().info(
            #         'Target detected! I gave coords to drone kamikadze'
            #     )

            now = time.time()
            fps = 1.0 / (now - self.last_time)
            self.last_time = now

            self.get_logger().info(
                f'Detections: {len(detections)} | FPS: {fps:.1f}'
            )

            for det in detections:
                x1, y1, x2, y2, conf, cls = det.tolist()
                label = f'{self.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    frame,
                    label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

            img_filename = os.path.join(self.output_dir, f'yolo_{self.img_counter:05d}.jpg')
            cv2.imwrite(img_filename, frame)
            self.img_counter += 1
            
        except Exception as e:
            self.get_logger().error(str(e))

        finally:
            self.busy = False


def main():
    rclpy.init()
    node = CNNDetectorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
