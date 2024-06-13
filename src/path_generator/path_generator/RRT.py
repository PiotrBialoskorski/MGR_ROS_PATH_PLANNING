import rclpy
from rclpy.node import Node

from my_interfaces.msg import EssentialData
import numpy as np
import random
import math

class tree_vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.origin = None

class RRT_path(Node):
    
    def __init__(self):
        super().__init__("Rapid_Random_Tree")
        self.subscriber = self.create_subscription(
            EssentialData, 
            "/EssData", 
            self.set_callback(),
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

    def grid_create(self):
        self.grid = np.reshape(self.data, (self.map_size[0],self.map_size[1]))

    def pos_to_pix(self):
        self.initial_pose = np.array([int(self.initial.x/self.resolution), int(self.initial.y/self.resolution)])
        self.goal_pose = np.array([int(self.goal.x/self.resolution), int(self.goal.y/self.resolution)])

    def path_generate(self):
        # self.grid[self.initial_pose[0]][self.initial_pose[1]] = 255
        # self.grid[self.goal_pose[0]][self.goal_pose[1]] = 250
        # self.threshhold = int(0.5/self.resolution)
        # self.flag = 0
        # self.dist = int(1/self.resolution)
        # while self.flag == 0:
        #     x = random.randint(0, self.map_size[0])
        #     y = random.randint(0, self.map_size[1])

        #     for i in range(0, self.threshhold):
        #         for j in range(0, self.threshhold):
        #             if self.grid[self.goal_pose[0] - i][self.goal_pose[1] - j] == 5:
        #                 self.flag = 1
        #             elif self.grid[self.goal_pose[0] + i][self.goal_pose[1] + j] == 5:
        #                 self.flag = 1
        #             elif self.grid[self.goal_pose[0] - i][self.goal_pose[1] + j] == 5:
        #                 self.flag = 1
        #             elif self.grid[self.goal_pose[0] + i][self.goal_pose[1] - j] == 5:
        #                 self.flag = 1
        #             else:
        #                 pass

        self.dD = 0.25 # [m]
        self.init_vertex = tree_vertex(self.initial_pose[0], self.initial_pose[1])
        self.goal_vertex = tree_vertex(self.goal_pose[0], self.goal_pose[1])
        self.tree_vertexes = [self.init_vertex]
    
    def random_vertex(self):
        return tree_vertex(random.rand()*self.map_size[0]*self.resolution,random.rand()*self.map_size[1]*self.resolution)
    
    def closest_vertex(self, rand_vertex):
        nearest_vertex = self.tree_vertexes[-1]
        min_dist = math.sqrt((rand_vertex.x - nearest_vertex.x)**2 + (rand_vertex.y - nearest_vertex.y)**2)
        for i in self.tree_vertexes:
            temp_dist = math.sqrt((rand_vertex.x - i.x)**2 + (rand_vertex.y - i.y)**2)
            if temp_dist < min_dist:
                min_dist = temp_dist
                nearest_vertex = i
        alpha = math.atan2(rand_vertex.x - nearest_vertex.x, rand_vertex.y - nearest_vertex.y)
        return i, alpha

    def create_vertex(self, nearest_vertex):
        return tree_vertex(nearest_vertex.x + self.dD)
def main(args=None):
    rclpy(args=args)

    RRT_PATH = RRT_path()

    rclpy.spin(RRT_PATH)
    
    RRT_PATH.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()