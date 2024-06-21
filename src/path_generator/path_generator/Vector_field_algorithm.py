import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from my_interfaces.msg import EssentialData
from nav_msgs.msg import Path as pth
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import math

class Vec_algorithm(Node):
    def __init__(self):
        super().__init__("Vector_field_algorithm")
        self.path_publisher = self.create_publisher(pth, '/path', 10)
        self.data_subscriber = self.create_subscription(
            EssentialData,
            '/EssData',
            self.VecField_callback,
            10
        )
        self.data_subscriber

    def VecField_callback(self, msg : EssentialData):
        self.resolution = msg.resolution
        self.data = msg.data
        self.initial_pose = msg.initial
        self.goal_pose = msg.goal
        self.map_size = np.array([msg.width, msg.height])
        self.grid = np.reshape(self.data, (self.map_size[1],self.map_size[0])) # grid[y][x] 
        self.get_logger().info("Resolution, occupancy grid, initial and goal point received")
        self.Vector_field_path()

    def Vector_field_path(self):
        path = [np.array([self.initial_pose.y, self.initial_pose.x])]
        goal = np.array([self.goal_pose.y, self.goal_pose.x])
        D = 1.
        norm = 0.15
        res = math.sqrt((path[0][0] - goal[0])**2 + (path[0][1] - goal[1])**2)
        new_pose = path[0]
        k = 0
        while res >= 0.1:
            vec = goal - new_pose
            vec_norm = math.sqrt((vec[0])**2 + (vec[1])**2)
            scale = norm / vec_norm
            vec = vec * scale
            iter_x = int(new_pose[1] / self.resolution)
            iter_y = int(new_pose[0] / self.resolution)
            dist = D
            repel_vec = np.array([0.0,0.0])
            for i in range(int(-D/self.resolution), int(D/self.resolution)+1):
                for j in range(int(-D/self.resolution), int(D/self.resolution)+1):
                    if self.grid[iter_y + i][iter_x + j] == 100:
                        temp_dist = math.sqrt((new_pose[0] - (iter_y + i)*self.resolution)**2 + (new_pose[1] - (iter_x + j)*self.resolution)**2)
                        if temp_dist <= D:
                            repel = new_pose - np.array([(iter_y + i) * self.resolution, (iter_x + j) * self.resolution])
                            repel_scale = (0.1 / dist) * ((1.0/temp_dist - 1.0 /D)**2)
                            repel_vec = repel_vec + repel * repel_scale
            if (((-0.001 <= vec[0] + repel_vec[0] <= 0.001) and (-0.001 <= vec[1] + repel_vec[1] <= 0.001)) or math.sqrt(repel_vec[0]**2 + repel_vec[1]**2) > 10):
                print("Stuck in local minimum")
                break
            new_pose = new_pose + vec + repel_vec
            path.append(new_pose)
            res = vec_norm
            if(k >= 10000):
                break
            k = k + 1      
        self.send_path(path)
    
    def send_path(self, path):
        path_msg = pth()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_poses = []
        for i in path:
            path_pose = PoseStamped()
            path_pose.pose.position.x = i[1]
            path_pose.pose.position.y = i[0]
            path_pose.pose.position.z = 0.0
            path_pose.header.frame_id = "base_link"
            path_pose.header.stamp = self.get_clock().now().to_msg()
            path_poses.append(path_pose)
        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Publishing path")

def main(args=None):
    rclpy.init(args=args)

    Vec_algs = Vec_algorithm()

    rclpy.spin(Vec_algs)

    Vec_algs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()