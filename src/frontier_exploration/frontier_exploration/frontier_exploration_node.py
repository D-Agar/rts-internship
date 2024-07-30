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
        
    
    def map_callback(self, msg):
        # Get all Occupancy grid's data as attributes
        self.map_data = msg
    
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
