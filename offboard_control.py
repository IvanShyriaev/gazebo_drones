import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool


class OffboardCircle(Node):
    def __init__(self):
        super().__init__('offboard_circle')

        self.pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        # self.pub_coords = self.create_publisher(
        #     PoseStamped,
        #     '/mavros/setpoint_position/to_destroy',
        #     10
        # )

        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.radius = 11150.0        # метри
        self.height = 125.0        # висота польоту
        self.angular_speed = 0.3 # рад/с

        self.pose = PoseStamped()

        self.timer = self.create_timer(0.1, self.publish_setpoint)

        self.counter = 0
        self.start_time = self.get_clock().now()

    def publish_setpoint(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        self.pose.pose.position.x = self.radius * math.cos(self.angular_speed * t)
        self.pose.pose.position.y = self.radius * math.sin(self.angular_speed * t)
        self.pose.pose.position.z = self.height

        self.pose.header.stamp = now.to_msg()
        self.pub.publish(self.pose)

        self.counter += 1

        if self.counter == 20:
            self.set_offboard_mode()

        if self.counter == 30:
            self.arm()

    def set_offboard_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('SetMode service not available')
            return

        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.mode_client.call_async(req)

        self.get_logger().info('OFFBOARD enabled')

    def arm(self):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arming service not available')
            return

        req = CommandBool.Request()
        req.value = True
        self.arm_client.call_async(req)

        self.get_logger().info('Vehicle armed')


def main():
    rclpy.init()
    node = OffboardCircle()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
