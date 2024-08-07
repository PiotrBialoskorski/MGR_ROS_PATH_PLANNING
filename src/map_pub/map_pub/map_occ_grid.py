import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from my_interfaces.msg import MapOccData

import cv2
import numpy as np

class MapOccPub(Node):

    def __init__(self):
        super().__init__('OccPub')
        self.data = [0]
        self.resolution = 0.
        self.width = 0
        self.height = 0
        self.publisher_ = self.create_publisher(MapOccData, '/OccGrid', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription
        
    def listener_callback(self, map_info : OccupancyGrid):
        self.get_logger().info(f"Map information received")
        self.data = map_info.data
        self.resolution = map_info.info.resolution
        self.width = map_info.info.width
        self.height = map_info.info.height
        self.sending()

    def sending(self):
        timer_period = 5
        self.timer =self.create_timer(timer_period, self.send_msg)
        
    def send_msg(self):
        msg_test = MapOccData()
        msg_test.data = self.data
        msg_test.resolution = self.resolution
        msg_test.width = self.width
        msg_test.height = self.height
        self.publisher_.publish(msg_test)
        self.get_logger().info('Publishing Occupancy Grid')

def main(args=None):
    rclpy.init(args=args)

    mappub = MapOccPub()
    rclpy.spin(mappub)
    mappub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()