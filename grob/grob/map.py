import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid


class Map():
    def __init__(self):

        # Key things that we will use later
        self.raw_map_data = 0
        self.map_grid = 0
        self.resolution = 0.05

        # In cartesian coords
        self.orig_x = 0
        self.orig_y = 0

        # List of obstacle coordinates (i, j)\
        # TESTING CODE FOR TAKING GAZEBO OUT THE LOOP
        obstacleX = [37, 39, 41, 42, 43, 45, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 78, 79, 80, 81, 83, 85, 87]
        obstacleY = [39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39]
        self.obstacle_list = [obstacleX, obstacleY]

    # Call to update map when a new occupancy grid message arrives
    def update_map(self, map_msg: OccupancyGrid):

        self.map_grid = np.reshape(map_msg.data, (-1, map_msg.info.width))

        self.resolution = map_msg.info.resolution
        self.orig_x = map_msg.info.origin.position.x
        self.orig_y = map_msg.info.origin.position.y

        # Reset obstacle lists
        self.obstacle_list = []
        obstacleX = []
        obstacleY = []
        for j in range(0, map_msg.info.height):
            for i in range(0, map_msg.info.width):
                if (self.map_grid[j][i] >= 1):
                    obstacleX.append(i)
                    obstacleY.append(j)

        self.obstacle_list = [obstacleX, obstacleY]

        # obstacleX = self.obstacle_list[0]
        # obstacleY = self.obstacle_list[1]
        # plt.scatter(obstacleX, obstacleY)
        # plt.show()

    def index_to_cartersian(self, point_to_map):
        # Take in the indexes of a point (i, j) and map it to cartersian coordinate (x, y)
        x = point_to_map[0] * self.resolution + self.orig_x
        y = point_to_map[1] * self.resolution + self.orig_y

        return [x, y]

    def index_to_catersian_x(self, index_x):
        # Map just an index to an cartesian point along the x axis
        x = index_x * self.resolution + self.orig_x

        return x

    def index_to_catersian_y(self, index_y):
        # Map just an index to an cartesian point along the y axis
        y = index_y * self.resolution + self.orig_y

        return y

    def cartesian_to_index_x(self, x):
        # Map cartesian point along the x axis to an index
        idx_x = (x - self.orig_x) / self.resolution

        return idx_x

    def cartesian_to_index_y(self, y):
        # Map cartesian point along the y axis to an index
        idx_y = (y - self.orig_y) / self.resolution

        return idx_y