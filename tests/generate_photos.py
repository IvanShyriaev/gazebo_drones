import rclpy
import sys
import math
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool


class SimulateDroneFlightForDataset(Node):
    def __init__(self, namespace=''):
        super().__init__('fly_to_generate_photos')
        self.ns = f'/{namespace}' if namespace else ''
        
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f'{self.ns}/setpoint_position/local',
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

        self.counter = 0
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.get_logger().info(f'Dataset flight generator started. namespace: {self.ns if self.ns else "none"}')

    def timer_cb(self):
        self.counter += 1
        self.get_logger().info(f'Flight simulation step: {self.counter}')

    def set_offboard_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return
        
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(req)

    def arm(self):
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available')
            return
        
        req = CommandBool.Request()
        req.value = True
        self.arm_client.call_async(req)


def main():
    rclpy.init()
    namespace = sys.argv[1] if len(sys.argv) > 1 else ''
    
    node = SimulateDroneFlightForDataset(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

