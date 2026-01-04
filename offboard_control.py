# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

# class OffboardControl(Node):
#     def __init__(self):
#         super().__init__('multi_offboard_control')

#         # Список дронів та їх цільових висот (Z у PX4 від'ємна, тому -3.0 це вгору)
#         self.drones = {
#             'px4_1': {'height': -3.0, 'id': 1},
#             'px4_2': {'height': -2.0, 'id': 2},
#         }

#         self.publishers_offboard_mode = {}
#         self.publishers_trajectory = {}
#         self.publishers_vehicle_command = {}

#         # Створюємо паблішери для кожного дрона
#         for ns in self.drones.keys():
#             qos_profile = QoSProfile(
#                 reliability=ReliabilityPolicy.BEST_EFFORT,
#                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
#                 history=HistoryPolicy.KEEP_LAST,
#                 depth=1
#             )

#             self.publishers_offboard_mode[ns] = self.create_publisher(OffboardControlMode, f'/{ns}/fmu/in/offboard_control_mode', qos_profile)
#             self.publishers_trajectory[ns] = self.create_publisher(TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', qos_profile)
#             self.publishers_vehicle_command[ns] = self.create_publisher(VehicleCommand, f'/{ns}/fmu/in/vehicle_command', qos_profile)

#         self.offboard_setpoint_counter = 0
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def publish_vehicle_command(self, ns, command, **params):
#         msg = VehicleCommand()
#         msg.command = command
#         msg.param1 = params.get("param1", 0.0)
#         msg.param2 = params.get("param2", 0.0)
#         msg.target_system = self.drones[ns]['id']
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.publishers_vehicle_command[ns].publish(msg)

#     def arm(self, ns):
#         self.publish_vehicle_command(ns, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
#         self.get_logger().info(f"{ns} armed")

#     def set_offboard_mode(self, ns):
#         self.publish_vehicle_command(ns, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
#         self.get_logger().info(f"{ns} set to offboard mode")

#     def timer_callback(self):
#         for ns, data in self.drones.items():
#             # 1. Публікуємо Heartbeat (OffboardControlMode), щоб PX4 не вийшов з Offboard
#             offboard_msg = OffboardControlMode()
#             offboard_msg.position = True
#             offboard_msg.velocity = False
#             offboard_msg.acceleration = False
#             offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#             self.publishers_offboard_mode[ns].publish(offboard_msg)

#             # 2. Відправляємо на цільову висоту
#             trajectory_msg = TrajectorySetpoint()
#             trajectory_msg.position = [0.0, 0.0, data['height']]
#             trajectory_msg.yaw = 0.0
#             trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#             self.publishers_trajectory[ns].publish(trajectory_msg)

#             # 3. Армінг та перехід в Offboard після короткої паузи (1 секунда)
#             if self.offboard_setpoint_counter == 10:
#                 self.arm(ns)
#                 self.set_offboard_mode(ns)

#         self.offboard_setpoint_counter += 1

# def main(args=None):
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, MountControl
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.mount_pub = self.create_publisher(MountControl, 'mavros/mount_control/command', 10)
        self.waypoint_reached_pub = self.create_publisher(Int32, 'waypoint_reached', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_callback, qos_profile)
        
        # Services
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.waypoints = [
            (7.47, 30.14, 30.0),
            (-203.06, -54.12, 40.0),
            (-209.86, 40.66, 40.0),
            (-63.66, -256.44, 40.0),
            (127.05, -323.76, 30.0)
        ]

        self.current_state = State()
        self.current_local_pose = None
        self.state_machine = "WAITING"
        self.current_wp_idx = 0
        
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 10.0
        self.target_yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Offboard Control Node started. Camera control active.")

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_local_pose = msg

    def set_camera_down(self):
        """Направляє камеру вниз"""
        mount_msg = MountControl()
        mount_msg.header.stamp = self.get_clock().now().to_msg()
        mount_msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        mount_msg.pitch = -90.0
        mount_msg.roll = 0.0
        mount_msg.yaw = 0.0
        self.mount_pub.publish(mount_msg)

    def control_loop(self):
        if self.current_local_pose is None:
            return

        self.set_camera_down()

        if self.state_machine == "WAITING":
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                self.state_machine = "NAVIGATING"
            else:
                self.set_offboard_mode()
                self.arm()

        elif self.state_machine == "NAVIGATING":
            target = self.waypoints[self.current_wp_idx]
            self.target_x, self.target_y, self.target_z = target
            
            dx = target[0] - self.current_local_pose.pose.position.x
            dy = target[1] - self.current_local_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            self.target_yaw = math.atan2(dy, dx)

            if dist < 3.0:
                # Повідомляємо іншу ноду про досягнення точки
                msg = Int32()
                msg.data = self.current_wp_idx
                self.waypoint_reached_pub.publish(msg)
                self.get_logger().info(f"Waypoint {self.current_wp_idx} reached")
                
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.state_machine = "LANDING"

        elif self.state_machine == "LANDING":
            self.set_land_mode()

        self.publish_setpoint()

    def publish_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        sy, cy = math.sin(self.target_yaw * 0.5), math.cos(self.target_yaw * 0.5)
        msg.pose.orientation.z, msg.pose.orientation.w = sy, cy
        self.pub.publish(msg)

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
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()