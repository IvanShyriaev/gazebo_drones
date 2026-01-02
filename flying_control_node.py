import rclpy
import sys
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool


class FlyingTargetController(Node):
    def __init__(self, namespace=''):
        super().__init__('attack_drone')
    
        self.ns = f'/{namespace}' if namespace else ''
    
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f'{self.ns}/setpoint_position/local',
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            f'{self.ns}/target/pose',
            self.target_cb,
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            f'{self.ns}/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            f'{self.ns}/set_mode'
        )

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0
        self.pose.pose.orientation.w = 1.0

        self.have_target = False
        self.counter = 0
        self.offboard_enabled = False
        self.armed = False

        self.timer = self.create_timer(0.1, self.publish_setpoint)

        self.get_logger().info(f'Attack drone OFFBOARD node started. namespace: {self.ns if self.ns else "none"}')

    def target_cb(self, msg: PoseStamped):
        self.pose.pose.position.x = msg.pose.position.x
        self.pose.pose.position.y = msg.pose.position.y
        self.pose.pose.position.z = msg.pose.position.z

        self.have_target = True

        self.get_logger().info(
            f'Target received -> '
            f'x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, '
            f'z={msg.pose.position.z:.2f}'
        )

    def publish_setpoint(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = 'map'
        self.setpoint_pub.publish(self.pose)
        self.counter += 1

        if self.counter == 20 and not self.offboard_enabled:
            self.set_offboard_mode()
            self.offboard_enabled = True

        if self.counter == 30 and not self.armed:
            self.arm()
            self.armed = True

    def set_offboard_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return
        
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(req)
        self.get_logger().info('OFFBOARD mode requested')

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
    node = FlyingTargetController(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
