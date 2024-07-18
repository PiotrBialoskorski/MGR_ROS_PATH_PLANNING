import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from my_interfaces.msg import EssentialData
from nav_msgs.msg import Path as pth
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import math
import time

class Path_length(Node):
    def __init__(self):
        super().__init__("Getting_length_of_path")
        self.len_subscriber = self.create_subscription(
            pth,
            '/path',
            self.length_callback,
            10
        )
        self.len_subscriber

    def length_callback(self, msg : pth):
        self.get_logger().info("Path received")
        path = []
        len = 0
        poses = msg.poses

        for point in poses:
            x = point.pose.position.x
            y = point.pose.position.y
            p = [x,y]
            path.append(p)
        print(np.shape(path))
        for i in range(1, np.shape(path)[0]-1):
            dist = math.sqrt((path[i-1][0] - path[i][0])**2 + (path[i-1][1] - path[i][1])**2)
            len += dist
        print(f"Total length of path: {len}")

def main(args=None):
    rclpy.init(args=args)

    pth_len = Path_length()

    rclpy.spin(pth_len)

    pth_len.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()