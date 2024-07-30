#!/usr/bin/env python3

from time import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
import tf2_ros
import numpy as np

class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node') # type: ignore
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.init_map = True
        self.frontiers = []
        
    
    def map_callback(self, msg):
        if self.init_map:
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.origin_theta = msg.info.origin.orientation.w
            self.init_map = False
        
        # Get all Occupancy grid's data as attributes
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        # Map grid is is row-major order, starting with (0, 0)
        self.grid = np.asarray(msg.data).reshape((self.map_height, self.map_width))
        
        self.frontiers = self.get_frontiers()
        
        # Sanity check: no frontiers found
        if self.frontiers is not None:
            frontier = self.select_frontier()
            self.go_to_frontier(frontier)
        else:
            self.get_logger().info("No more frontiers, environment explored")
            self.shutdown()
    
    def get_frontiers(self):
        frontiers = []
        # Search through OccupancyGrid
        for y in range(0, self.map_height-1):
            for x in range(0, self.map_width-1):
                # Free space is 0
                if self.grid[y, x] == 0:
                    # Check to see if there is an unknown cell in a 3x3 radius of chosen cell
                    if -1 in self.grid[x-2:x+2, y-2:y+2]:
                        frontiers.append((x, y))
        return frontiers


    def select_frontier(self):
        pass
    
    
    def go_to_frontier(self, frontier):
        pass
    
    
    
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        
        
    
    # Find frontiers
    # Select frontier ()

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
