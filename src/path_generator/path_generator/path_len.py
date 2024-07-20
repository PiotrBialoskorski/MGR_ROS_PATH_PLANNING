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
import statistics

class vec():
    def __init__(self, vector, p1, p2, d, res):
        self.vector = vector
        self.vx = vector[0]
        self.vy = vector[1]
        self.p1 = p1
        self.p2 = p2
        self.i1 = [int(p1[0]/res),int(p1[1]/res)]
        self.i2 = [int(p2[0]/res),int(p2[1]/res)]
        self.dist = d
class Path_length(Node):
    def __init__(self):
        super().__init__("Getting_length_of_path")
        self.len_subscriber = self.create_subscription(
            pth,
            '/path',
            self.length_callback,
            10
        )

        self.map_info = self.create_subscription(
            EssentialData,
            '/EssData',
            self.map_info_callback,
            10,
        )
        self.len_subscriber
        self.map_info
    def map_info_callback(self, msg : EssentialData):
        self.resolution = msg.resolution
        self.data = msg.data
        self.map_size = np.array([msg.width,msg.height])
        self.get_logger().info("Resolution and occupancy grid received")
        self.grid = np.reshape(self.data, (self.map_size[1],self.map_size[0])) # grid looks like this grid[y][x]

    def length_callback(self, msg : pth):
        self.get_logger().info("Path received")
        self.path = []
        self.len = 0
        poses = msg.poses

        for point in poses:
            x = point.pose.position.x
            y = point.pose.position.y
            p = [x,y]
            self.path.append(p)
        vecs = []
        for i in range(1, np.shape(self.path)[0]):
            dist = math.sqrt((self.path[i-1][0] - self.path[i][0])**2 + (self.path[i-1][1] - self.path[i][1])**2)
            vecs.append(vec([self.path[i-1][0] - self.path[i][0],self.path[i-1][1] - self.path[i][1]], self.path[i], self.path[i-1], dist,self.resolution))
            self.len += dist
        
        safe_dist = 0.25 
        safe_wsp = []
        print(len(vecs))
        points = []
        for v in vecs:
            d_norm = [v.vector[0]/v.dist, v.vector[1]/v.dist]
            if v.dist <= self.resolution * 5:
                points.append(v.i1)
            else:
                for n in range(0, int(v.dist/(4*self.resolution))):
                    if n * 4 * self.resolution < v.dist:
                        points.append([v.i1[0] + int(n * 4 * d_norm[0]), v.i1[1] + int(n * 4 * d_norm[1])])
        points.append(vecs[-1].i2)
        print(len(points))
        for point in points:
            k = 0
            l = 0
            flag = 0
            while flag == 0:
                for i in range(-k, k + 1):
                    for j in range(-l, l + 1):    
                        if self.grid[i + point[1]][j + point[0]] != 0:
                            d = math.sqrt((i)**2 + (j)**2)
                            d = d * self.resolution
                            flag = 1
                            break
                    if flag == 1:
                        break
                k += 1
                l += 1
            safe_wsp.append(d/safe_dist)       
        print(f"Total length of path: {self.len}")
        print(f"Mean safe: {statistics.mean(safe_wsp)} Min safe: {min(safe_wsp)}")

def main(args=None):
    rclpy.init(args=args)

    pth_len = Path_length()

    rclpy.spin(pth_len)

    pth_len.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()