import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool


class SimulateDroneFlightForDataset(Node):
    def __init__(self):
        super().__init__('fly_to_generate_photos')

    