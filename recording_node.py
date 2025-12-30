import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import os
from datetime import datetime

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')

        # Налаштування QoS (як у твоїй ноді з YOLO)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_cb,
            qos
        )

        self.bridge = CvBridge()
        
        # Створюємо папку video, якщо її немає
        self.output_dir = os.path.join(os.getcwd(), 'video')
        os.makedirs(self.output_dir, exist_ok=True)

        # Генеруємо назву файлу на основі дати та часу
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_filename = os.path.join(self.output_dir, f'record_{timestamp}.mp4')

        # Параметри відео (ініціалізуються при першому кадрі)
        self.video_writer = None
        self.fps = 1.0  # Можна змінити залежно від камери
        
        self.get_logger().info(f'Recorder started. Waiting for images on "camera/image_raw"...')

    def image_cb(self, msg: Image):
        try:
            # Конвертуємо ROS Image в OpenCV формат
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Ініціалізація VideoWriter при отриманні першого кадру
            if self.video_writer is None:
                height, width, _ = frame.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Кодек для .mp4
                self.video_writer = cv2.VideoWriter(
                    self.video_filename, 
                    fourcc, 
                    self.fps, 
                    (width, height)
                )
                self.get_logger().info(f'Recording started: {self.video_filename} ({width}x{height})')

            # Записуємо кадр у файл
            self.video_writer.write(frame)

        except Exception as e:
            self.get_logger().error(f'Could not save frame: {str(e)}')

    def destroy_node(self):
        # Закриваємо файл при вимкненні ноди
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info('Video saved and file closed.')
        super().destroy_node()

def main():
    rclpy.init()
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping recording...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()