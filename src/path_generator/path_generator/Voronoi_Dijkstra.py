import rclpy
from rclpy.node import Node

from my_interfaces.msg import EssentialData
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as pth
from scipy.spatial import Voronoi

import numpy as np
import matplotlib.pyplot as plt
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
        self.id = None

class arc():
    def __init__(self, vertice1, vertice2, len):
        self.vertices = [vertice1, vertice2]
        self.len = len
        

class Dijkstra(Node):
    def __init__(self):
        super().__init__("Voronoi_graph_Dijkstra_method")
        self.path_pub = self.create_publisher(pth, "/path", 10)
        self.subscriber = self.create_subscription(
            EssentialData,
            "/EssData",
            self.graph_callback,
            10,
            )
        self.subscriber
    
    def graph_callback(self, msg : EssentialData):
        self.resolution = msg.resolution
        self.data = msg.data
        self.initial_pose = msg.initial
        self.goal_pose = msg.goal
        self.map_size = [msg.width, msg.height]
        self.grid = np.reshape(self.data, (self.map_size[1], self.map_size[0]))
        self.get_logger().info("Resolution, occupancy grid, initial and goal point received") 
        vertices = self.voronoi_diagram()
        self.Dijkstra(vertices)

    def voronoi_diagram(self):
        walls = []
        graph_vertecies = []
        for i in range(self.map_size[0]):
            for j in range(self.map_size[1]):
                black = 0
                if self.grid[j][i] != 0:
                    for k in range(-1,2):
                        for l in range(-1,2):
                            if 0 < j + k < self.map_size[1] and 0 < i + l < self.map_size[0]:    
                                if self.grid[j+k][i+l] !=0:
                                    black += 1
                    if black < 8:
                        walls.append([i,j])
        
        numerator = 0
        voronoi = Voronoi(walls)
        for vert in voronoi.vertices:
            if 0 < vert[0] < self.map_size[0] and 0 < vert[1] < self.map_size[1]:
                if self.grid[int(vert[1])][int(vert[0])] == 0:
                    node = vertice(vert[0], vert[1])
                    node.id = numerator
                    graph_vertecies.append(node)
            numerator += 1

        
        for pair in voronoi.ridge_vertices:
            if pair[0] != -1 and pair[1] != -1:
                flag_1 = 0
                flag_2 = 0
                for node in graph_vertecies:
                    if node.id == pair[0]:
                        temp_one = node
                        flag_1 = 1
                    elif node.id == pair[1]:
                        temp_two = node
                        flag_2 = 1
                    if flag_1 == 1 and flag_2 == 1:
                        break
                if flag_1 == 1 and flag_2 == 1:
                    dist = math.sqrt((temp_one.x - temp_two.x)**2 + (temp_one.y - temp_two.y)**2)
                    new_arc = arc(temp_one, temp_two, dist)
                    temp_one.arcs.append(new_arc)
                    temp_two.arcs.append(new_arc)

        starting_node = vertice(int(self.initial_pose.x/self.resolution), int(self.initial_pose.y/self.resolution))
        goal_node = vertice(int(self.goal_pose.x/self.resolution), int(self.goal_pose.y/self.resolution))
        start_dist = math.sqrt((starting_node.x - graph_vertecies[0].x)**2 + ( starting_node.y - graph_vertecies[0].y)**2)
        end_dist = math.sqrt((goal_node.x - graph_vertecies[0].x)**2 + (goal_node.y - graph_vertecies[0].y)**2)
        end_node = graph_vertecies[0]
        start_node = graph_vertecies[0]
        for node in graph_vertecies:
            new_start_dist = math.sqrt((starting_node.x - node.x)**2 + ( starting_node.y - node.y)**2)
            new_end_dist = math.sqrt((goal_node.x - node.x)**2 + (goal_node.y - node.y)**2)
            temp_start = node
            temp_end = node
            if new_start_dist < start_dist:
                start_node = temp_start
                start_dist = new_start_dist
            if new_end_dist < end_dist:
                end_node = temp_end
                end_dist = new_end_dist
        start_arc = arc(starting_node, start_node, start_dist)
        end_arc = arc(goal_node, end_node, end_dist)
        starting_node.arcs.append(start_arc)
        start_node.arcs.append(start_arc)
        goal_node.arcs.append(end_arc)
        end_node.arcs.append(end_arc) 
        graph_vertecies.insert(0, starting_node)
        graph_vertecies.append(goal_node)
    
        return graph_vertecies
    
    @profile
    def Dijkstra(self, vertices):
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
                            temp_val = curr_ver.value + ar.len
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

    Graph = Dijkstra()

    rclpy.spin_once(Graph)

    Graph.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()