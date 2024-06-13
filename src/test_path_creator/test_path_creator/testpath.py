import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from my_interfaces.msg import MapOccData
import matplotlib.pyplot as plt

import cv2
import numpy as np

class map_test(Node):

    def __init__(self):
        super().__init__('MapTest')
        self.map = cv2.imread('/home/roslab/ros2_mgr/maps/map.pgm')
        self.data = [0]
        self.resolution = 0.
        self.subscription = self.create_subscription(
            MapOccData,
            '/test',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, map_info : MapOccData):
        self.data = map_info.data
        self.resolution = map_info.resolution
        self.get_logger().info('Data nad resolution received')
        self.rewrite()
        self.grid = np.reshape(self.data, (np.shape(self.map)[0],np.shape(self.map)[1]))

    def rewrite(self):
        self.path_map = np.ones((np.shape(self.map)[0],np.shape(self.map)[1],np.shape(self.map)[2]))
        self.path_map = self.path_map*255


def main(args=None):
    rclpy.init(args=args)

    test = map_test()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()