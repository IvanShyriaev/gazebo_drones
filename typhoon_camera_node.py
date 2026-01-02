import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TyphoonCamera(Node):
    def __init__(self, namespace=''):
        super().__init__('typhoon_camera')
        
        self.ns = f'/{namespace}' if namespace else ''
        
        self.pub = self.create_publisher(Image, f'{self.ns}/camera/image_raw', 10)
        self.bridge = CvBridge()

        pipeline = (
            "udpsrc port=5600 ! "
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open video stream")

        self.timer = self.create_timer(0.03, self.timer_cb)  # ~30 FPS
        
        self.get_logger().info(f'Typhoon Camera node started with namespace: {self.ns if self.ns else "none"}')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

def main():
    rclpy.init()
    
    import sys
    namespace = sys.argv[1] if len(sys.argv) > 1 else ''
    
    node = TyphoonCamera(namespace=namespace)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

