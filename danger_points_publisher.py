import rclpy
from rclpy.node import Node
import math
import pandas as pd
import folium
import os
import numpy as np
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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

        # Список для зберігання даних точок (підтверджені небезпеки)
        self.danger_points_data = []
        # Список для зберігання YOLO детекцій (потенційні небезпеки)
        self.yolo_detections_data = []
        
        self.current_gps = None

        # Параметри кластеризації (eps в метрах)
        self.cluster_eps_meters = 10.0  # точки в радіусі 10 метрів об'єднуються
        self.cluster_min_samples = 1    # мінімум точок для кластера

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            'mavros/global_position/global', 
            self.gps_callback, 
            qos_profile
        )
        
        # Підтверджені небезпечні точки (waypoint_reached)
        self.waypoint_reached_sub = self.create_subscription(
            Int32, 
            'waypoint_reached', 
            self.waypoint_reached_callback, 
            10
        )

        # YOLO детекції (потенційні небезпеки)
        self.yolo_detection_sub = self.create_subscription(
            Int32,
            'yolo_detection',
            self.yolo_detection_callback,
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

        self.get_logger().info("Danger Map Logger Node started with DBSCAN clustering...")

    def gps_callback(self, msg):
        self.current_gps = msg

    def save_camera_image(self, idx, prefix='danger'):
        if self.last_frame is None:
            self.get_logger().warn("No camera frame available")
            return None

        img_dir = os.path.expanduser("~/danger_images")
        os.makedirs(img_dir, exist_ok=True)

        img_path = f"{img_dir}/{prefix}_{idx}.png"
        cv2.imwrite(img_path, self.last_frame)

        return img_path

    def image_callback(self, msg):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warn(f"Image convert failed: {e}")

    def yolo_detection_callback(self, msg):
        """Обробка YOLO детекції - потенційна небезпека"""
        detection_idx = msg.data
        self.save_yolo_detection(detection_idx)

    def save_yolo_detection(self, idx):
        """Зберігає YOLO детекцію як потенційну небезпеку"""
        if self.current_gps is None or math.isnan(self.current_gps.latitude):
            self.get_logger().warn(f"Cannot save YOLO detection {idx}: GPS not available")
            return

        img_path = self.save_camera_image(idx, prefix='yolo_detection')

        self.yolo_detections_data.append({
            'id': idx,
            'lat': self.current_gps.latitude,
            'lon': self.current_gps.longitude,
            'alt': self.current_gps.altitude,
            'image': img_path
        })

        self.get_logger().info(
            f"YOLO detection {idx} saved as potential threat"
        )

        self.generate_final_map()

    def waypoint_reached_callback(self, msg):
        """Обробка повідомлення про досягнення точки - підтверджена небезпека"""
        waypoint_idx = msg.data
        self.publish_danger_mark(waypoint_idx)

    def publish_danger_mark(self, idx):
        """Зберігає підтверджену небезпечну точку"""
        if self.current_gps is None or math.isnan(self.current_gps.latitude):
            self.get_logger().warn(f"Cannot save point {idx}: GPS not available")
            return

        img_path = self.save_camera_image(idx, prefix='confirmed_danger')

        self.danger_points_data.append({
            'id': idx,
            'lat': self.current_gps.latitude,
            'lon': self.current_gps.longitude,
            'alt': self.current_gps.altitude,
            'image': img_path
        })

        self.get_logger().info(
            f"Confirmed danger point {idx} saved"
        )

        self.generate_final_map()

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Обчислює відстань між двома GPS координатами в метрах"""
        R = 6371000  # Радіус Землі в метрах
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    def cluster_points(self, points_data):
        """Кластеризує точки за допомогою DBSCAN"""
        if len(points_data) == 0:
            return {}

        # Конвертуємо GPS координати в метри для DBSCAN
        coords = np.array([[p['lat'], p['lon']] for p in points_data])
        
        # Апроксимація: 1 градус широти ≈ 111км
        # Для більшої точності можна використати UTM проекцію
        lat_center = np.mean(coords[:, 0])
        coords_meters = coords.copy()
        coords_meters[:, 0] *= 111000  # lat to meters
        coords_meters[:, 1] *= 111000 * math.cos(math.radians(lat_center))  # lon to meters
        
        # DBSCAN кластеризація
        clustering = DBSCAN(
            eps=self.cluster_eps_meters,
            min_samples=self.cluster_min_samples,
            metric='euclidean'
        ).fit(coords_meters)
        
        labels = clustering.labels_
        
        # Групуємо точки по кластерах
        clusters = {}
        for idx, label in enumerate(labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(points_data[idx])
        
        return clusters

    def generate_final_map(self):
        """Створює інтерактивну мапу з кластеризованими мітками"""
        if not self.danger_points_data and not self.yolo_detections_data:
            self.get_logger().warn("No data to generate map.")
            return

        # Визначаємо центр мапи
        if self.danger_points_data:
            center_lat = self.danger_points_data[0]['lat']
            center_lon = self.danger_points_data[0]['lon']
        elif self.yolo_detections_data:
            center_lat = self.yolo_detections_data[0]['lat']
            center_lon = self.yolo_detections_data[0]['lon']
        else:
            return

        # Створюємо мапу
        m = folium.Map(
            location=[center_lat, center_lon], 
            zoom_start=18, 
            tiles='OpenStreetMap'
        )
        
        # Кластеризуємо підтверджені небезпеки
        danger_clusters = self.cluster_points(self.danger_points_data)
        
        for cluster_id, cluster_points in danger_clusters.items():
            cluster_size = len(cluster_points)
            
            # Центр кластера
            center_lat = np.mean([p['lat'] for p in cluster_points])
            center_lon = np.mean([p['lon'] for p in cluster_points])
            avg_alt = np.mean([p['alt'] for p in cluster_points])
            
            # Формуємо HTML для popup
            images_html = ""
            point_ids = []
            for p in cluster_points:
                point_ids.append(str(p['id']))
                if p['image']:
                    images_html += f'<img src="file://{p["image"]}" width="200" style="margin: 5px;"><br>'
            
            html = f"""
            <b style="color: red; font-size: 16px;">⚠️ CONFIRMED DANGER</b><br>
            <b>Points: {cluster_size}</b> (IDs: {', '.join(point_ids)})<br>
            Avg Altitude: {avg_alt:.1f} m<br><br>
            {images_html}
            """

            # Базовий розмір іконки + 1 піксель за кожну додаткову детекцію
            icon_size = 38 + (cluster_size - 1)
            
            # Створюємо кастомну іконку зі збільшеним розміром
            icon_html = f'''
                <div style="font-size: {icon_size}px;">
                    <i class="fa fa-exclamation-triangle" style="color: red;"></i>
                </div>
            '''
            
            folium.Marker(
                location=[center_lat, center_lon],
                popup=folium.Popup(html, max_width=450),
                icon=folium.DivIcon(html=icon_html, icon_size=(icon_size, icon_size))
            ).add_to(m)

        # Кластеризуємо YOLO детекції
        yolo_clusters = self.cluster_points(self.yolo_detections_data)
        
        for cluster_id, cluster_points in yolo_clusters.items():
            cluster_size = len(cluster_points)
            
            # Центр кластера
            center_lat = np.mean([p['lat'] for p in cluster_points])
            center_lon = np.mean([p['lon'] for p in cluster_points])
            avg_alt = np.mean([p['alt'] for p in cluster_points])
            
            # Формуємо HTML для popup
            images_html = ""
            point_ids = []
            for p in cluster_points:
                point_ids.append(str(p['id']))
                if p['image']:
                    images_html += f'<img src="file://{p["image"]}" width="200" style="margin: 5px;"><br>'
            
            html = f"""
            <b style="color: orange; font-size: 16px;">🔍 YOLO DETECTION</b><br>
            <i>Potential threats detected</i><br>
            <b>Detections: {cluster_size}</b> (IDs: {', '.join(point_ids)})<br>
            Avg Altitude: {avg_alt:.1f} m<br><br>
            {images_html}
            """

            # Базовий розмір іконки + 1 піксель за кожну додаткову детекцію
            icon_size = 38 + (cluster_size - 1)
            
            # Створюємо кастомну іконку зі збільшеним розміром
            icon_html = f'''
                <div style="font-size: {icon_size}px;">
                    <i class="fa fa-eye" style="color: orange;"></i>
                </div>
            '''
            
            folium.Marker(
                location=[center_lat, center_lon],
                popup=folium.Popup(html, max_width=450),
                icon=folium.DivIcon(html=icon_html, icon_size=(icon_size, icon_size))
            ).add_to(m)

        # Зберігаємо мапу
        map_path = os.path.expanduser('~/mission_danger_map.html')
        m.save(map_path)
        
        total_danger_clusters = len(danger_clusters)
        total_yolo_clusters = len(yolo_clusters)
        
        self.get_logger().info(
            f"Map updated: {map_path} | "
            f"Danger clusters: {total_danger_clusters} ({len(self.danger_points_data)} points) | "
            f"YOLO clusters: {total_yolo_clusters} ({len(self.yolo_detections_data)} points)"
        )


def main():
    rclpy.init()
    node = DangerMapLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.generate_final_map()
        node.get_logger().info("Final map generated on shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()