import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

import torch
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
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.sub = self.create_subscription(
            Image,
            f'{self.ns}/camera/image_raw',
            self.image_cb,
            qos
        )

        self.target_pub = self.create_publisher(PoseStamped, f'{self.ns}/target/pose', 10)
        self.detection_pub = self.create_publisher(Int32, 'yolo_detection', 10)

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO('/home/remote_gazebo/ros2_ws/src/typhoon_camera/typhoon_camera/best.pt')
        self.model.to(self.device)

        self.busy = False
        self.last_time = time.time()
        self.img_counter = 0
        self.detection_counter = 0

        self.output_dir = './yolo_outputs'
        os.makedirs(self.output_dir, exist_ok=True)

    def image_cb(self, msg: Image):
        if self.busy:
            return

        self.busy = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model.track(
                frame,
                imgsz=640,
                conf=0.6,
                persist=True,
                tracker="botsort.yaml",
                verbose=False
            )

            boxes = results[0].boxes

            if boxes is not None and len(boxes) > 0:
                # Беремо перший знайдений об'єкт
                box = boxes[0]

                track_id = int(box.id.item()) if box.id is not None else -1

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose.position.x = 10.0
                pose_msg.pose.position.y = 10.0
                pose_msg.pose.position.z = 2.0
                pose_msg.pose.orientation.w = 1.0
                self.target_pub.publish(pose_msg)

                detection_msg = Int32()
                detection_msg.data = track_id
                self.detection_pub.publish(detection_msg)

                self.get_logger().info(f'Tracking ID: {track_id} | Total Detections: {self.detection_counter}')
                self.detection_counter += 1

            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())

                t_id = int(box.id.item()) if box.id is not None else "N/A"
                label = f'ID:{t_id} {self.model.names[cls]} {conf:.2f}'

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            now = time.time()
            fps = 1.0 / (now - self.last_time)
            self.last_time = now
            self.get_logger().info(f'FPS: {fps:.1f}')

            img_filename = os.path.join(self.output_dir, f'yolo_{self.img_counter:05d}.jpg')
            cv2.imwrite(img_filename, frame)
            self.img_counter += 1

        except Exception as e:
            self.get_logger().error(f"Error in image_cb: {str(e)}")
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