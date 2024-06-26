import rclpy
from rclpy.node import Node

from my_interfaces.msg import EssentialData
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as pth

import numpy as np
import matplotlib.pyplot as plt
import math
import cv2

class face_edges():
    def __init__(self, x1_i, y1_j, x2_i, y2_j, res):
        self.x1_iter = x1_i
        self.y1_iter = y1_j
        self.x2_iter = x2_i
        self.y2_iter = y2_j
        self.res = res
        self.P1 = [x1_i * self.res, y1_j * self.res]
        self.P2 = [x2_i * self.res, y2_j * self.res]

class Voronoi_graph(Node):
    def __init__(self):
        super().__init__("Graph_methods")
        # self.graph_pub = self.create_publisher(pth, "/path", 10)
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
        # self.voronoi_edges()
        self.voronoi_diagram()
    
    def voronoi_diagram(self):
        temp_grid = self.grid
        temp_grid[temp_grid == 100] = 255
        temp_grid[temp_grid == 0] = 0
        temp_grid[temp_grid == -1] = 127
        temp_grid = np.array(temp_grid, dtype = np.uint8)
        rect = (0,0, self.map_size[1], self.map_size[0])
        subdiv = cv2.Subdiv2D(rect)

    # def map_2d_filtered(self):
    #     new_grid = np.ones((self.map_size[1], self.map_size[0]))
    #     new_grid = new_grid * np.inf
    #     corners = []
    #     walls = []
    #     x2_test = []
    #     y2_test = []
    #     for i in range(1, self.map_size[0]-1):
    #         for j in range(1, self.map_size[1]-1):
    #             if ((self.grid[j][i] == 100)):
    #                 black_sum = 0
    #                 for k in range(-1,2):
    #                     for l in range(-1,2):
    #                         if self.grid[j+k][i+l] == 100:
    #                             black_sum = black_sum + 1
    #                 if ((black_sum <= 8)):
    #                     new_grid[j][i] = None
    #                     x2_test.append(i)
    #                     y2_test.append(j)
    #                     if ((self.grid[j + 1][i] == 100) and (self.grid[j][i + 1] == 100) and (self.grid[j - 1][i] == 0) and (self.grid[j][i - 1] == 0)):
    #                         corners.append([j,i])     
    #                     elif (((self.grid[j][i - 1] == 100) and (self.grid[j + 1][i] == 100) and (self.grid[j - 1][i] == 0) and (self.grid[j][i + 1] == 0))):
    #                         corners.append([j,i])  
    #                     elif ((self.grid[j][i - 1] == 100) and (self.grid[j - 1][i] == 100) and (self.grid[j + 1][i] == 0) and (self.grid[j][i + 1] == 0)):
    #                         corners.append([j,i])  
    #                     elif (((self.grid[j - 1][i] == 100) and (self.grid[j][i + 1]) and (self.grid[j + 1][i] == 0) and (self.grid[j][i - 1] == 0))):
    #                         corners.append([j,i])  
    #                     elif (((self.grid[j + 1][i] == 100) and (self.grid[j - 1][i] == 100) and (self.grid[j][i + 1] == 100) and (self.grid[j][i - 1] == 100))):
    #                         corners.append([j,i])  
    #                     elif (((self.grid[j + 1][i] == 100) and (self.grid[j + 1][i + 1] == 0) and (self.grid[j][i + 1] == 100) and (self.grid[j][i - 1] == 100) and (self.grid[j + 1][i - 1] == 100))):
    #                         corners.append([j,i])  
    #                     elif (((self.grid[j + 1][i] == 100) and (self.grid[j + 1][i - 1] == 0) and (self.grid[j][i + 1] == 100) and (self.grid[j][i - 1] == 100) and (self.grid[j + 1][i + 1] == 100))):
    #                         corners.append([j,i])  
    #     for i in range(len(corners)-1):
    #         for j in range(i,len(corners)-1):
    #             if ((corners[i][0] == corners[j][0]) or (corners[i][1] == corners[j][1])):
    #                 dir = [corners[j][0] - corners[i][0], corners[j][1] - corners[i][1]]
    #                 if ((dir[0] != 0) or (dir[1] != 0)):
    #                     if ((dir[0] == 0) and (dir[1] != 0)):
    #                         dir[1] = int(abs(dir[1]) / dir[1])
    #                     if ((dir[0] != 0) and (dir[1] == 0)):
    #                         dir[0] = int(abs(dir[0]) / dir[0])
    #                     flag = 0
    #                     iter_0 = 0
    #                     iter_1 = 0
    #                     while ((iter_0 <= abs(corners[j][0] - corners[i][0])) and (iter_1 <= abs(corners[j][1] - corners[i][1]))):
    #                         if new_grid[corners[j][0] + l * dir[0]][corners[i][1] + k * dir[1]] == np.inf:
    #                             flag = 1
    #                             break
    #                         if iter_1 <= abs(corners[j][1] - corners[i][1]):
    #                             iter_1 += 1
    #                         if iter_0 <= abs(corners[j][0] - corners[i][0]):
    #                             iter_0 += 1

    #                     if flag == 0:
    #                         walls.append(face_edges(corners[i][1], corners[i][0], corners[j][1], corners[j][0], self.resolution))

    #     return walls, new_grid, x2_test, y2_test

    # def voronoi_diagram(self):
    #     walls, grid2, x2_t, y2_t= self.map_2d_filtered()
    #     x_test = []
    #     y_test = []
    #     for wall in walls:
    #         wall_vec = np.array([wall.y2_iter - wall.y1_iter, wall.x2_iter - wall.x1_iter])
    #         wall_vec_norm = math.sqrt((wall_vec[0])**2 + (wall_vec[1])**2)
    #         for i in range(0, self.map_size[0]):
    #             for j in range(0, self.map_size[1]):
    #                 if self.grid[j][i] == 0:
    #                     point_vec = np.array([j - wall.y1_iter, i - wall.x1_iter])
    #                     proj_length = np.dot(wall_vec, point_vec) / wall_vec_norm
    #                     if proj_length < 0:
    #                         dist = math.sqrt((j - wall.y1_iter)**2 + (i - wall.x1_iter)**2)
    #                     if proj_length > wall_vec_norm:
    #                         dist = math.sqrt((j - wall.y2_iter)**2 + (i - wall.x2_iter)**2)
    #                     else:
    #                         proj_point = wall.P1 + proj_length*wall_vec/wall_vec_norm
    #                         dist = math.sqrt((point_vec[0] - proj_point[0])**2 + (point_vec[1] - proj_point[1])**2)
                        
    #                     if ((dist < grid2[j][i]) or (grid2[j][i] == np.inf)):
    #                         grid2[j][i] = dist
    #                     # vec1 = np.array([i - wall.x1_iter, j - wall.y1_iter])
    #                     # vec2 = np.array([i - wall.x2_iter, j - wall.y2_iter])
    #                     # vec3 = vec1 + vec2
    #                     # vec3 = vec3 / 2
    #                     # vec3_norm = math.sqrt((vec3[0])**2 + (vec3[1])**2)
    #                     # if vec3_norm < grid[j][i]:
    #                     #     grid[j][i] = vec3_norm
    #                     # elif vec3_norm == grid[j][i]:
    #                     #     x_test.append(i)
    #                     #     y_test.append(j)
    #     voronoi_lines = []
    #     for i in range(self.map_size[0]):
    #         for j in range(self.map_size[1]):
    #             if self.grid[j][i] == 0:
    #                 distances = []
    #                 for wall in walls:
    #                     point_vec = np.array([j - wall.y1_iter, i - wall.x1_iter])
    #                     proj_length = np.dot(wall_vec, point_vec) / wall_vec_norm
    #                     if proj_length < 0:
    #                         dist = np.linalg.norm(point_vec)
    #                     elif proj_length > wall_vec_norm:
    #                         dist = np.linalg.norm([j - wall.y2_iter, i - wall.x2_iter])
    #                     else:
    #                         proj_point = wall.P1 + proj_length * wall_vec / wall_vec_norm
    #                         dist = np.linalg.norm(point_vec - proj_point)
    #                     distances.append(dist)
    #                 distances.sort()
    #                 if len(distances) > 1 and abs(distances[0] - distances[1]) < 0.1:  # próg różnicy odległości
    #                     voronoi_lines.append((j, i))
    #                     x2_t.append(i)
    #                     y2_t.append(j)
    #     print(voronoi_lines)
    #     plt.scatter(x2_t,y2_t)
    #     plt.show

    #     # for i in range(1, self.map_size[0] - 1):
    #     #         for j in range(1, self.map_size[1] - 1):
    #     #             if ((self.grid[j][i] == 0) and (grid2[j][i] != None)) :
    #     #                 if ((grid2[j][i] > grid2[j - 1][i]) and (grid2[j][i] > grid2[j + 1][i])):
    #     #                     y2_t.append(j)
    #     #                     x2_t.append(i)
    #     #                 elif ((grid2[j][i] > grid2[j][i - 1]) and (grid2[j][i] > grid2[j][i + 1])):
    #     #                     y2_t.append(j)
    #     #                     x2_t.append(i)
    #     #                 elif ((grid2[j][i] > grid2[j - 1][i]) and (grid2[j][i] > grid2[j + 1][i])):
    #     #                     y2_t.append(j)
    #     #                     x2_t.append(i)
    #     # plt.scatter(y2_t, x2_t)
    #     # plt.show()


    # # def voronoi_edges(self):
    # #     x_test, y_test, obs = self.map_2d_filtered()
    # #     print("wchodze do morderczej petli")
    # #     Walls = obs
    # #     points = [(x,y) for x in range(min(x_test), max(x_test)) for y in range(min(y_test), max(y_test)) if self.grid[y][x] == 0]
    # #     print("checkpoint 2")
    # #     vor = Voronoi(points)
    # #     fig, ax = plt.subplots()
    # #     voronoi_plot = voronoi_plot_2d(vor)
    # #     print("checkpoint 3")
    # #     for point in Walls:
    # #         ax.plot(point[1], point[0], 'ro')
    # #     print("checkpoint 4")
    # #     ax.set_xlim([0, max(x_test)])
    # #     ax.set_ylim([0, max(y_test)])
    # #     print("checkpoint 5")
    # #     plt.gca().set_aspect('equal', adjustable = 'box')
    # #     plt.title("Diagram voronoi z uwzglednieniem przeszkod")
    # #     plt.gca().invert_yaxis()
    # #     plt.show()

    # #     graph = nx.Graph()
    # #     print("checkpoint 3")
    # #     for ridge_points in vor.ridge_vertices:
    # #         if -1 not in ridge_points:
    # #             p1 = tuple(vor.vertices[ridge_points[0]])
    # #             p2 = tuple(vor.vertices[ridge_points[1]])
    # #             if (0 <= p1[0] <= self.map_size[1] and 0 <= p1[1] <= self.map_size[0] and
    # #                 0 <= p2[0] <= self.map_size[1] and 0 <= p2[1] <= self.map_size[0]):
    # #                 graph.add_edge(p1,p2, weight = np.linalg.norm(np.array(p1) - np.array(p2)))
    # #     print("checkpoint 4")
    # #     # Start_node = (int(self.initial_pose.x/self.resolution), int(self.initial_pose.y/self.resolution))
    # #     # end_node = (int(self.goal_pose.x/self.resolution), int(self.goal_pose.y/self.resolution))
    # #     # start_node_closest = min(graph.nodes, key = lambda node: np.linalg.norm(np.array(node) - np.array(Start_node)))
    # #     # end_node_closest = min(graph.nodes, key = lambda node: np.linalg.norm(np.array(node) - np.array(end_node)))
    # #     # shortest_path = nx.shortest_path(graph, source = start_node_closest, target=end_node_closest, weight = "weight")
    # #     # print("checkpoint 5")
    # #     # fig, ax = plt.subplots()
    # #     # # nx.draw(graph, pos={node: node for node in graph.nodes()}, node_size=10, ax=ax)
    # #     # path_edges = list(zip(shortest_path, shortest_path[1:]))
    # #     # nx.draw_networkx_edges(graph, pos={node: node for node in graph.nodes()}, edgelist=path_edges, edge_color='r', width=2, ax=ax)
    # #     # plt.title('Najkrótsza ścieżka na grafie Voronoi')
    # #     # plt.gca().invert_yaxis()  # Odwróć oś y, aby dopasować do konwencji map
    # #     # plt.show()
    # #     # print("checkpoint 6")
    # #     fig2,ax = plt.subplots()
    # #     print("checkpoint 6")
    # #     nx.draw(graph, pos={node: node for node in graph.nodes()}, node_size = 10, ax = ax)
    # #     plt.title("Graf Voronoi")
    # #     plt.show()
    
def main(args=None):
    rclpy.init(args=args)

    Graph = Voronoi_graph()

    rclpy.spin(Graph)
    
    Graph.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()