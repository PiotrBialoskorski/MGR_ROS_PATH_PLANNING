import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as pth
from my_interfaces.msg import EssentialData

import matplotlib.pyplot as plt
import numpy as np
import math
import cv2
import time
import os
import psutil

def process_memory():
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss

def profile(main):
    def wrapper(*args, **kwargs):
        mem_before = process_memory()
        result = main(*args, **kwargs)
        mem_after = process_memory()
        print("{}: consumed memory: {:,}".format(main.__name__, mem_before, mem_after - mem_before))

        return result
    return wrapper

class vertice():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.value = np.inf
        self.arcs = []
        self.parent = None

class arc():
    def __init__(self, vertice1, vertice2, len):
        self.vertices = [vertice1, vertice2]
        self.len = len
        
class visibility_graph_methods(Node):
    def __init__(self):
        super().__init__("Visibility_Graph_A_star_Algorithm")
        self.path_pub = self.create_publisher(pth, '/path', 10)
        self.data_sub = self.create_subscription(
            EssentialData,
            '/EssData',
            self.method_callback,
            10
        )
        self.data_sub

    def method_callback(self, msg : EssentialData):
        self.data = msg.data
        self.resolution = msg.resolution
        self.initial_pose = msg.initial
        self.goal_pose = msg.goal
        self.map_size = np.array([msg.width, msg.height])
        self.grid = np.reshape(self.data, (self.map_size[1],self.map_size[0])) # grid looks like this grid[y][x]
        self.get_logger().info("Resolution, occupancy grid, initial and goal point received")
        arcs, vertices = self.Vis_graph()
        self.A_star(arcs, vertices)

    def Vis_graph(self):
        d = int(0.25/self.resolution)
        temp_grid = self.grid
        temp_grid[temp_grid == 100] = 255
        temp_grid[temp_grid == 0] = 0
        temp_grid[temp_grid == -1] = 255
        kernel = np.ones((2 * d + 1,2 * d + 1),np.uint8)
        temp_grid = np.array(temp_grid, dtype = np.uint8)
        temp_grid_dilated = cv2.dilate(temp_grid, kernel, iterations = 1)
        corners = cv2.goodFeaturesToTrack(temp_grid_dilated, 500, 0.01, 10)
        corners = np.int0(corners)
        cor = []
        cor.append([int(self.initial_pose.x/self.resolution), int(self.initial_pose.y/self.resolution)])
        for i in corners:
            x1,y1 = i.ravel()
            cor.append([x1,y1])
        cor.append([int(self.goal_pose.x/self.resolution), int(self.goal_pose.y/self.resolution)])
        vertices = [vertice(x,y) for x,y in cor]
        arcs = []
        for i in range(0, len(vertices)):
            for j in range(i, len(vertices)):
                if vertices[i] != vertices[j]:
                    flag = 0
                    vec = np.array([vertices[j].x - vertices[i].x, vertices[j].y - vertices[i].y])
                    D = math.sqrt((vec[0])**2 + (vec[1])**2)
                    dx = abs(vec[0])
                    dy = abs(vec[1])
                    sx = 1 if vec[0] > 0 else -1
                    sy = 1 if vec[1] > 0 else -1
                    err = dx - dy
                    x0 = vertices[i].x
                    y0 = vertices[i].y
                    x1 = vertices[j].x
                    y1 = vertices[j].y
                    while True:
                        if x0 == x1 and y0 == y1:
                            break
                        if self.grid[y0][x0] != 0:
                            flag = 1
                            break
                        e2 = err * 2
                        if e2 > -dy:
                            err -= dy
                            x0 += sx
                        if e2 < dx:
                            err += dx
                            y0 += sy
                    if flag == 0:
                        new_arc = arc(vertices[i], vertices[j], D)
                        vertices[i].arcs.append(new_arc)
                        vertices[j].arcs.append(new_arc)
                        arcs.append(new_arc)
        return arcs,vertices
    
    @profile
    def A_star(self, arcs, vertices):
        start = time.time()
        vertices[0].value = 0 
        goal_vertice = vertices[-1]
        Q = vertices
        Used = []
        iterator = 0
        while Q != []:
            Q_val = np.inf
            for i in Q: 
                if i.value < Q_val and i not in Used:
                    curr_ver = i
                    Q_val = i.value
            Q.remove(curr_ver)
            Used.append(curr_ver)
            for ar in curr_ver.arcs:
                for ver in ar.vertices:
                    if ver != curr_ver:
                        if ver.value != 0:
                            temp_val = curr_ver.value + ar.len + self.Heurestic_function(ver, goal_vertice)
                            if temp_val < ver.value:
                                ver.value = temp_val
                                ver.parent = curr_ver
            iterator += 1
            if goal_vertice.value != np.inf:
                break
        
        print(f"Number of iterations: {iterator}")
        path = []
        active_ver = goal_vertice
        while active_ver is not None:
            path.append(active_ver)
            active_ver = active_ver.parent
        
        self.send_path(path)
        end = time.time()
        print(f"time of just algorithm: {end - start}")
    
    def Heurestic_function(self, current_ver, goal_ver):
        x = goal_ver.x - current_ver.x
        y = goal_ver.y - current_ver.y
        vec = np.array([x,y])
        vec_len = math.sqrt((vec[0])**2 + (vec[1])**2)
        return vec_len

    def send_path(self, path):
        path_msg = pth()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_poses = []
        for i in path:
            path_pose = PoseStamped()
            path_pose.pose.position.x = i.x * self.resolution
            path_pose.pose.position.y = i.y * self.resolution
            path_pose.pose.position.z = 0.0
            path_pose.header.frame_id = "base_link"
            path_pose.header.stamp = self.get_clock().now().to_msg()
            path_poses.append(path_pose)
        path_msg.poses = path_poses
        self.path_pub.publish(path_msg)
        self.get_logger().info("Publishing path")



def main(args=None):
    rclpy.init(args=args)


    Vis_Graph_meth = visibility_graph_methods()
    
    rclpy.spin_once(Vis_Graph_meth)


    Vis_Graph_meth.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
