import rclpy
from rclpy.node import Node
import math
import pandas as pd
import folium
import os

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class DangerMapLogger(Node):
    def __init__(self):
        super().__init__('danger_map_logger')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Список для зберігання даних точок
        self.danger_points_data = []
        self.current_gps = None

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            'mavros/global_position/global', 
            self.gps_callback, 
            qos_profile
        )
        
        self.waypoint_reached_sub = self.create_subscription(
            Int32, 
            'waypoint_reached', 
            self.waypoint_reached_callback, 
            10
        )

        self.bridge = CvBridge()
        self.last_frame = None

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )


        self.get_logger().info("Danger Map Logger Node started. Waiting for waypoint notifications...")

    def gps_callback(self, msg):
        self.current_gps = msg

    def save_camera_image(self, idx):
        if self.last_frame is None:
            self.get_logger().warn("No camera frame available")
            return None

        img_dir = os.path.expanduser("~/danger_images")
        os.makedirs(img_dir, exist_ok=True)

        img_path = f"{img_dir}/danger_{idx}.png"
        cv2.imwrite(img_path, self.last_frame)

        return img_path

    def image_callback(self, msg):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warn(f"Image convert failed: {e}")


    def waypoint_reached_callback(self, msg):
        """Обробка повідомлення про досягнення точки"""
        waypoint_idx = msg.data
        self.publish_danger_mark(waypoint_idx)

    def publish_danger_mark(self, idx):
        if self.current_gps is None or math.isnan(self.current_gps.latitude):
            self.get_logger().warn(f"Cannot save point {idx}: GPS not available")
            return

        img_path = self.save_camera_image(idx)

        self.danger_points_data.append({
            'id': idx,
            'lat': self.current_gps.latitude,
            'lon': self.current_gps.longitude,
            'alt': self.current_gps.altitude,
            'image': img_path
        })

        self.get_logger().info(
            f"Danger point {idx} saved (image={'OK' if img_path else 'NONE'})"
        )

        self.generate_final_map()

    def generate_final_map(self):
        """Створює інтерактивну мапу через Pandas та Folium"""
        if not self.danger_points_data:
            self.get_logger().warn("No data to generate map.")
            return

        df = pd.DataFrame(self.danger_points_data)
        
        # Створюємо мапу з центром на першій точці
        m = folium.Map(
            location=[df.lat.iloc[0], df.lon.iloc[0]], 
            zoom_start=18, 
            tiles='OpenStreetMap'
        )
        
        # Додаємо маркери для кожної точки
        for _, row in df.iterrows():
            html = f"""
            <b>Danger Object {int(row.id)}</b><br>
            Altitude: {row.alt:.1f} m<br><br>
            <img src="file://{row.image}" width="320">
            """

            folium.Marker(
                location=[row.lat, row.lon],
                popup=folium.Popup(html, max_width=350),
                icon=folium.Icon(color='red', icon='exclamation-triangle', prefix='fa')
            ).add_to(m)

        # Зберігаємо мапу
        map_path = os.path.expanduser('~/mission_danger_map.html')
        m.save(map_path)
        self.get_logger().info(f"Map updated: {map_path} ({len(self.danger_points_data)} points)")

    def generate_fake_image(self, idx, lat, lon, alt):
        """
        Створює тестове зображення з текстом (ID, координати)
        """
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)

        cv2.putText(img, f"DANGER OBJECT {idx}", (40, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.putText(img, f"Lat: {lat:.6f}", (40, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.putText(img, f"Lon: {lon:.6f}", (40, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.putText(img, f"Alt: {alt:.1f} m", (40, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(img, timestamp, (40, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        img_dir = os.path.expanduser("~/danger_images")
        os.makedirs(img_dir, exist_ok=True)

        img_path = f"{img_dir}/danger_{idx}.png"
        cv2.imwrite(img_path, img)

        return img_path


def main():
    rclpy.init()
    node = DangerMapLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.generate_final_map()  # Генеруємо фінальну мапу при перериванні
        node.get_logger().info("Final map generated on shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()