import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, MountControl
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PVODiscreteFlight(Node):
    def __init__(self):
        super().__init__('pvo_discrete_flight')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Паблішери
        self.pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.mount_pub = self.create_publisher(MountControl, 'mavros/mount_control/command', 10)
        
        # Підписники
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_callback, qos_profile)
        
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # Точки (X, Y, Z)
        self.waypoints = [
            (7.47, 30.14, 30.0),
            (-203.06, -54.12, 40.0),
            (-209.86, 40.66, 40.0),
            (-63.66, -256.44, 40.0),
            (127.05, -323.76, 30.0)
        ]

        self.current_state = State()
        self.current_local_pose = None
        self.home_pose = None
        self.state_machine = "WAITING"
        self.current_wp_idx = 0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 10.0
        self.target_yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_local_pose = msg
        if self.home_pose is None:
            self.home_pose = msg.pose.position
            self.get_logger().info('Home position fixed.')

    def set_camera_down(self):
        """Направляє камеру вертикально вниз"""
        mount_msg = MountControl()
        mount_msg.header.stamp = self.get_clock().now().to_msg()
        mount_msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        mount_msg.pitch = -90.0 # Дивитися вниз
        mount_msg.roll = 0.0
        mount_msg.yaw = 0.0
        self.mount_pub.publish(mount_msg)

    def control_loop(self):
        if self.home_pose is None or self.current_local_pose is None:
            return

        now = self.get_clock().now()
        self.set_camera_down() # Постійно тримаємо камеру вниз

        if self.state_machine == "WAITING":
            self.target_x, self.target_y = self.home_pose.x, self.home_pose.y
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                self.state_machine = "NAVIGATING"
                self.get_logger().info("Starting waypoint mission")
            else:
                self.set_offboard_mode()
                self.arm()

        elif self.state_machine == "NAVIGATING":
            target = self.waypoints[self.current_wp_idx]
            self.target_x, self.target_y, self.target_z = target
            
            # Обчислюємо відстань до поточної цілі
            dx = target[0] - self.current_local_pose.pose.position.x
            dy = target[1] - self.current_local_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)

            # Розрахунок Yaw, щоб дрон дивився куди летить
            self.target_yaw = math.atan2(dy, dx)

            if dist < 3.0: # Радіус досягнення точки (3 метри)
                self.get_logger().info(f"Reached point {self.current_wp_idx + 1}")
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.state_machine = "LANDING"

        elif self.state_machine == "LANDING":
            self.set_land_mode()

        # Публікація позиції
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.pose.position.x = self.target_x
        pose_msg.pose.position.y = self.target_y
        pose_msg.pose.position.z = self.target_z
        
        # Перетворення Yaw в кватерніон
        sy = math.sin(self.target_yaw * 0.5)
        cy = math.cos(self.target_yaw * 0.5)
        pose_msg.pose.orientation.z = sy
        pose_msg.pose.orientation.w = cy
        
        self.pub.publish(pose_msg)

    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(req)

    def set_land_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.mode_client.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arm_client.call_async(req)

def main():
    rclpy.init()
    node = PVODiscreteFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()