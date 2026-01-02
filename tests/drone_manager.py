import rclpy
from rclpy.node import Node


class DroneManager(Node):
    def __init__(self, namespace=''):
        super().__init__('drone_manager')
        self.ns = f'/{namespace}' if namespace else ''
        self.get_logger().info(f'DroneManager initialized with namespace: {self.ns if self.ns else "none"}')


def main():
    rclpy.init()
    
    import sys
    namespace = sys.argv[1] if len(sys.argv) > 1 else ''
    
    node = DroneManager(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

