import rclpy
import sys
import math
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, MountControl
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PVODiscreteFlight(Node):
    def __init__(self, namespace=''):
        super().__init__('pvo_discrete_flight')

        self.ns = f'/{namespace}' if namespace else ''

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub = self.create_publisher(PoseStamped, f'{self.ns}/setpoint_position/local', 10)
        self.mount_pub = self.create_publisher(MountControl, f'{self.ns}/mount_control/command', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(State, f'{self.ns}/state', self.state_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, f'{self.ns}/local_position/pose', self.pose_callback, qos_profile)
        
        # Services
        self.arm_client = self.create_client(CommandBool, f'{self.ns}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'{self.ns}/set_mode')

        # Waypoints (X, Y, Z)
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
        self.target_z = 2.0
        self.target_yaw = 0.0
        
        self.setpoint_counter = 0
        self.debug_counter = 0

        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Namespace: {self.ns if self.ns else "none"}')
        self.get_logger().info(f'Subscribing to state: {self.ns}/state')
        self.get_logger().info(f'Subscribing to pose: {self.ns}/local_position/pose')

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_local_pose = msg
        if self.home_pose is None:
            self.home_pose = msg.pose.position
            self.target_x = self.home_pose.x
            self.target_y = self.home_pose.y
            self.get_logger().info(f'Home position fixed: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f}, {self.home_pose.z:.2f})')

    def set_camera_down(self):
        """Point camera straight down"""
        mount_msg = MountControl()
        mount_msg.header.stamp = self.get_clock().now().to_msg()
        mount_msg.mode = 2
        mount_msg.pitch = -90.0
        mount_msg.roll = 0.0
        mount_msg.yaw = 0.0
        self.mount_pub.publish(mount_msg)

    def publish_setpoint(self):
        """Publish setpoint to drone"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.target_x
        pose_msg.pose.position.y = self.target_y
        pose_msg.pose.position.z = self.target_z
        
        sy = math.sin(self.target_yaw * 0.5)
        cy = math.cos(self.target_yaw * 0.5)
        pose_msg.pose.orientation.z = sy
        pose_msg.pose.orientation.w = cy
        
        self.pub.publish(pose_msg)
        self.setpoint_counter += 1

    def control_loop(self):
        # Debug output every 3 seconds
        self.debug_counter += 1
        if self.debug_counter >= 30:
            self.get_logger().info(f'home_pose: {self.home_pose}, current_local_pose: {self.current_local_pose is not None}')
            self.get_logger().info(f'State - guided: {self.current_state.guided}, armed: {self.current_state.armed}')
            self.get_logger().info(f'State machine: {self.state_machine}')
            self.debug_counter = 0
        
        if self.home_pose is None or self.current_local_pose is None:
            return

        self.set_camera_down()
        self.publish_setpoint()

        if self.state_machine == "WAITING":
            if self.setpoint_counter < 100:
                return
            
            if not self.current_state.guided:
                self.set_offboard_mode()
            
            if self.current_state.guided and not self.current_state.armed:
                self.arm()
            
            if self.current_state.guided and self.current_state.armed:
                self.state_machine = "NAVIGATING"
                self.get_logger().info("Starting waypoint mission")

        elif self.state_machine == "NAVIGATING":
            target = self.waypoints[self.current_wp_idx]
            self.target_x, self.target_y, self.target_z = target
            
            dx = target[0] - self.current_local_pose.pose.position.x
            dy = target[1] - self.current_local_pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)

            self.target_yaw = math.atan2(dy, dx)

            if dist < 3.0:
                self.get_logger().info(f"Reached point {self.current_wp_idx + 1}")
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.state_machine = "LANDING"

        elif self.state_machine == "LANDING":
            self.set_land_mode()

    def set_offboard_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return
        
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(req)
        self.get_logger().info('OFFBOARD mode requested')

    def set_land_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return
        
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.mode_client.call_async(req)
        self.get_logger().info('LAND mode requested')

    def arm(self):
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available')
            return
        
        req = CommandBool.Request()
        req.value = True
        self.arm_client.call_async(req)
        self.get_logger().info('Arming requested')


def main():
    rclpy.init()
    namespace = sys.argv[1] if len(sys.argv) > 1 else ''
    
    node = PVODiscreteFlight(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()