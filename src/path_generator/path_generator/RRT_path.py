import rclpy
from rclpy.node import Node

from my_interfaces.msg import EssentialData
from nav_msgs.msg import Path as pth
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import numpy as np
import random
import math

class tree_vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.z = 0.0
        self.origin = None

class RRT_path(Node):
    
    def __init__(self):
        super().__init__("Rapid_Random_Tree")
        self.path_publisher = self.create_publisher(pth, '/path', 10)
        self.subscriber = self.create_subscription(
            EssentialData, 
            "/EssData", 
            self.set_callback,
            10,
            )
        self.subscriber
    
    def set_callback(self, msg : EssentialData):
        self.resolution = msg.resolution
        self.data = msg.data
        self.initial = msg.initial
        self.goal = msg.goal
        self.map_size = np.array([msg.width,msg.height])
        self.get_logger().info("Resolution, occupancy grid, initial and goal point received")
        self.grid_create()
        self.path_generate()
        self.send_path()

    def grid_create(self):
        self.grid = np.reshape(self.data, (self.map_size[1],self.map_size[0])) # grid looks like this grid[y][x]

    def path_generate(self):
        self.dD = 0.5 # [m]
        i = 0
        self.rand_tree = []
        self.init_vertex = tree_vertex(self.initial.x, self.initial.y)
        self.goal_vertex = tree_vertex(self.goal.x, self.goal.y)
        self.tree_vertexes = [self.init_vertex]
        while i <= 10000:
            rand_vertex = self.random_vertex()
            self.rand_tree.append(rand_vertex)
            closest_vertex = self.closest_vertex(rand_vertex)
            new_vertex = self.create_vertex(closest_vertex)
            if self.link_check(closest_vertex, new_vertex) == 0:
                self.tree_vertexes.append(new_vertex)
                i = i + 1
                if math.sqrt((new_vertex.x - self.goal_vertex.x)**2 + (new_vertex.y - self.goal_vertex.y)**2) <= self.dD:
                    self.goal_vertex.origin = new_vertex
                    self.tree_vertexes.append(self.goal_vertex)
                    break
        self.path = self.get_path(self.tree_vertexes)
    
    def random_vertex(self):
        return tree_vertex(random.random()*(self.map_size[0] - 1)*self.resolution,random.random()*(self.map_size[1] - 1)*self.resolution)
    
    def closest_vertex(self, rand_vertex):
        nearest_vertex = self.tree_vertexes[0]
        min_dist = math.sqrt((rand_vertex.x - nearest_vertex.x)*(rand_vertex.x - nearest_vertex.x) + (rand_vertex.y - nearest_vertex.y)*(rand_vertex.y - nearest_vertex.y))
        for i in self.tree_vertexes:
            temp_dist = math.sqrt((rand_vertex.x - i.x)*(rand_vertex.x - i.x) + (rand_vertex.y - i.y)*(rand_vertex.y - i.y))
            if temp_dist < min_dist:
                min_dist = temp_dist
                nearest_vertex = i
        self.res = [rand_vertex.x - nearest_vertex.x, rand_vertex.y - nearest_vertex.y]
        self.res[0] = self.res[0]/math.sqrt((rand_vertex.x - nearest_vertex.x)**2 + (rand_vertex.y - nearest_vertex.y)**2)
        self.res[1] = self.res[1]/math.sqrt((rand_vertex.x - nearest_vertex.x)**2 + (rand_vertex.y - nearest_vertex.y)**2)
        return nearest_vertex
    
    def create_vertex(self, nearest_vertex):
        new_temp = tree_vertex(nearest_vertex.x + self.dD * self.res[0], nearest_vertex.y + self.dD * self.res[1])
        new_temp.origin = nearest_vertex
        return new_temp
    
    def link_check(self, nearest_vertex, new_vertex):
        nearest_vertex_pix = [int(nearest_vertex.x/self.resolution), int(nearest_vertex.y/self.resolution)]
        new_vertex_pix = [int(new_vertex.x/self.resolution), int(new_vertex.y/self.resolution)]
        flag_obstacle = 0
        for i in range(min(new_vertex_pix[0], nearest_vertex_pix[0])-5, max(new_vertex_pix[0], nearest_vertex_pix[0])+5):
            for j in range(min(new_vertex_pix[1], nearest_vertex_pix[1])-5, max(new_vertex_pix[1], nearest_vertex_pix[1])+5):
                if ((self.grid[j][i] == 100)):
                        flag_obstacle = 1
                        return flag_obstacle
        return flag_obstacle
    
    def get_path(self, tree):
        tree_path = []
        vertex = tree[-1]
        while vertex is not None:
            tree_path.append(vertex)
            vertex = vertex.origin
        return tree_path


    def send_path(self):
        path_msg = pth()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_poses = []
        iter = 0
        for i in self.path:
            path_pose = PoseStamped()
            path_pose.pose.position.x = i.x
            path_pose.pose.position.y = i.y
            path_pose.pose.position.z = i.z
            path_pose.header.frame_id = "base_link"
            path_pose.header.stamp = self.get_clock().now().to_msg()
            path_poses.append(path_pose)
        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Publishing path")
        
    def check_grid(self):
        x_check = []
        y_check = []
        for i in range(0, self.map_size[0]):
            for j in range(0, self.map_size[1]):
                if self.grid[j][i] == 0:
                    x_check.append(i)
                    y_check.append(j)
        plt.scatter(x_check, y_check)
        plt.show()
        
                
def main(args=None):
    rclpy.init(args=args)

    RRT_PATH = RRT_path()

    rclpy.spin(RRT_PATH)
    
    RRT_PATH.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()