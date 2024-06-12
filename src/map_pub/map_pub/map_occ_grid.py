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
        self.map = cv2.imread('/home/roslab/mgr/maps/map.pgm')
        self.data = [0]
        self.resolution = 0.
        self.publisher_ = self.create_publisher(MapOccData, '/test', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription
        
    def listener_callback(self, map_info : OccupancyGrid):
        self.get_logger().info(f"Map information received: {map_info.info.resolution}")
        self.data = map_info.data
        self.resolution = map_info.info.resolution
        self.sending()

    def sending(self):
        timer_period = 1
        self.timer =self.create_timer(timer_period, self.send_msg)
        
    def send_msg(self):
        #msg_info.data = self.Occ(self.map)
        msg_test = MapOccData()
        msg_test.data = self.data
        msg_test.resolution = self.resolution
        self.publisher_.publish(msg_test)
        self.get_logger().info('Publishing Occupancy Grid')

    
    # def Occ(self,map):
    #     sizex = np.shape(map)[0]
    #     sizey = np.shape(map)[1]
    #     data = []
    #     for i in range(0, sizey):
    #         for j in range(0, sizex):
    #             if map[j,i,0] == 0:
    #                 data.append(100)
    #             else:
    #                 data.append(0)
    #     return data


def main(args=None):
    rclpy.init(args=args)

    mappub = MapOccPub()
    rclpy.spin(mappub)
    mappub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()