import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from my_interfaces.msg import EssentialData
from nav_msgs.msg import Path as pth
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import math

class Dyf_algorithm(Node):
    def __init__(self):
        super().__init__("Dyfusion_Algorithm")
        self.path_publisher = self.create_publisher(pth, "/path", 10)
        self.data_subscriber = self.create_subscription(
            EssentialData,
            '/EssData',
            self.Dyf_callback,
            10
        )
        self.data_subscriber

    def Dyf_callback(self, msg : EssentialData):
        self.data = msg.data
        self.resolution = msg.resolution
        self.initial_pose = msg.initial
        self.goal_pose = msg.goal
        self.map_size = np.array([msg.width, msg.height])
        self.grid = np.reshape(self.data, (self.map_size[1],self.map_size[0])) # grid looks like this grid[y][x]
        self.get_logger().info("Resolution, occupancy grid, initial and goal point received")
        self.Dyf_take_two()
        # self.dyfusion()

    # def dyfusion(self):
    #     work_grid = self.new_grid()
    #     initial = [int(self.initial_pose.y/self.resolution), int(self.initial_pose.x/self.resolution)]
    #     goal = [int(self.goal_pose.y/self.resolution), int(self.goal_pose.x/self.resolution)]
    #     mask = np.array([[math.sqrt(2), 1, math.sqrt(2)],[1, 0, 1],[math.sqrt(2), 1, math.sqrt(2)]])
    #     work_grid[initial[0],initial[1]] = 0.1
    #     pose = [initial[0], initial[1]]
    #     last_pose = pose
    #     counter = 0
    #     # while work_grid[goal[0]][goal[1]] == 0:
    #     for n in range(4):
    #         for i in range(-1,2):
    #             for j in range(-1,2):
    #                 if work_grid[pose[0] + i, pose[1] + j] != None:
    #                     if last_pose != [pose[0] + i, pose[1] + j]:
    #                         if ((work_grid[pose[0] + i, pose[1] + j] == 0) or (work_grid[pose[0] + i, pose[1] + j] > work_grid[pose[0], pose[1]] + mask[1 + i, 1 + j])):
    #                             work_grid[pose[0] + i, pose[1] + j] = work_grid[pose[0], pose[1]] + mask[1 + i, 1 + j]
    #                             lowest_cell = work_grid[pose[0] + i, pose[1] + j]
    #                             new_pose = [pose[0] + i, pose[1] + j]
    #         for k in range(-1, 2):
    #             for l in range(-1, 2):
    #                 if work_grid[pose[0] + k, pose[1] + l] != None:
    #                     if [last_pose[0], last_pose[1]] != [pose[0] + k, pose[1] + l]:
    #                         print(last_pose, [pose[0]+k,pose[1]+l])
    #                         if((k != 0) or (l != 0)):
    #                             temp = work_grid[pose[0] + k, pose[1] + l]
    #                             if temp < lowest_cell:
    #                                 lowest_cell = temp
    #                                 new_pose = [pose[0] + k, pose[1] + l]
    #         last_pose = pose
    #         pose = new_pose
    #         print(last_pose, pose, lowest_cell)
    #         if new_pose == goal:
    #             break
    #         # print(f"{counter}.huh")
    #     print("wyszedlem z petli")
    #     for i in range(-5,6):
    #         test = []
    #         for j in range(-5,6):
    #             test.append(round(work_grid[initial[0]+i,initial[1]+j],3))
    #         print(test)
    #     # path = self.get_path(work_grid, goal)

    def Dyf_take_two(self):
        work_grid = self.new_grid()
        initial = [int(self.initial_pose.y/self.resolution), int(self.initial_pose.x/self.resolution)]
        goal = [int(self.goal_pose.y/self.resolution), int(self.goal_pose.x/self.resolution)]
        mask = np.array([[math.sqrt(2), 1, math.sqrt(2)],[1, 0, 1],[math.sqrt(2), 1, math.sqrt(2)]])
        work_grid[initial[0],initial[1]] = 0
        V = [initial]
        while work_grid[goal[0],goal[1]] == np.inf:
            new_V = []
            for vertiece in V:
                for i in range(-1,2):
                    for j in range(-1,2):
                        if work_grid[vertiece[0] + i][vertiece[1] + j] != None:
                            if 0 < work_grid[vertiece[0] + i][vertiece[1] + j] <= np.inf:
                                if work_grid[vertiece[0]][vertiece[1]] + mask[1 + i][1 + j] < work_grid[vertiece[0] + i][vertiece[1] + j]:
                                    work_grid[vertiece[0] + i][vertiece[1] + j] = work_grid[vertiece[0]][vertiece[1]] + mask[1 + i][1 + j]
                                    new_vertiece = [vertiece[0] + i, vertiece[1] + j]
                                    new_V.append(new_vertiece)
            V = new_V
        path = self.get_path(work_grid, goal, initial)
        self.send_path(path)

    def get_path(self, grid, goal, initial):
        path = [goal]
        pose = goal
        lowest_cell = grid[pose[0]][pose[1]]
        while pose != initial:
            for i in range(-1,2):
                for j in range(-1,2):
                    if grid[pose[0] + i, pose[1] + j] != None:
                        if((i != 0) or (j != 0)):
                            temp = grid[pose[0] + i, pose[1] + j]
                            if temp <= lowest_cell:
                                lowest_cell = temp
                                new_pose = [pose[0] + i, pose[1] + j]
            pose = new_pose
            path.append(new_pose)
        return path
    
    def new_grid(self):
        grid_dyf = np.ones((self.map_size[1], self.map_size[0]))
        grid_dyf = grid_dyf * np.inf
        for i in range(self.map_size[0]):
            for j in range(self.map_size[1]):
                if self.grid[j][i] != 0:
                    for k in range(-2,3):
                        for l in range(-2,3):
                            grid_dyf[j+k][i+l] = None
        return grid_dyf
    
    def send_path(self, path):
        path_msg = pth()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_poses = []
        for i in path:
            path_pose = PoseStamped()
            path_pose.pose.position.x = i[1] * self.resolution
            path_pose.pose.position.y = i[0] * self.resolution
            path_pose.pose.position.z = 0.0
            path_pose.header.frame_id = "base_link"
            path_pose.header.stamp = self.get_clock().now().to_msg()
            path_poses.append(path_pose)
        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Publishing path")

def main(args=None):
    rclpy.init(args=args)

    Dyf_alg = Dyf_algorithm()

    rclpy.spin(Dyf_alg)

    Dyf_alg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()