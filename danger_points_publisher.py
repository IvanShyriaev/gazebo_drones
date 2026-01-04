import rclpy
from rclpy.node import Node
import math
import pandas as pd
import folium
import os

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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

        self.get_logger().info("Danger Map Logger Node started. Waiting for waypoint notifications...")

    def gps_callback(self, msg):
        self.current_gps = msg

    def waypoint_reached_callback(self, msg):
        """Обробка повідомлення про досягнення точки"""
        waypoint_idx = msg.data
        self.publish_danger_mark(waypoint_idx)

    def publish_danger_mark(self, idx):
        """Зберігає danger mark для поточної GPS позиції"""
        if self.current_gps is None or math.isnan(self.current_gps.latitude):
            self.get_logger().warn(f"Cannot save point {idx}: GPS data not available")
            return

        # Зберігаємо в список для Pandas
        self.danger_points_data.append({
            'id': idx,
            'lat': self.current_gps.latitude,
            'lon': self.current_gps.longitude,
            'alt': self.current_gps.altitude
        })
        self.get_logger().info(f"Danger point {idx} saved: lat={self.current_gps.latitude:.6f}, lon={self.current_gps.longitude:.6f}")
        
        # Генеруємо мапу після кожної точки (можна робити тільки в кінці)
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
            folium.Marker(
                location=[row.lat, row.lon],
                popup=f"Danger Object {int(row.id)}<br>Alt: {row.alt:.2f}m",
                icon=folium.Icon(color='red', icon='exclamation-triangle', prefix='fa')
            ).add_to(m)

        # Зберігаємо мапу
        map_path = os.path.expanduser('~/mission_danger_map.html')
        m.save(map_path)
        self.get_logger().info(f"Map updated: {map_path} ({len(self.danger_points_data)} points)")

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