#!/usr/bin/env python3

from copy import deepcopy
from math import floor, sqrt
import math
import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
from tf2_ros import Buffer, TransformListener
import numpy as np
from rclpy.duration import Duration
from sklearn.cluster import DBSCAN
from scipy.spatial import distance


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node') # type: ignore
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.init_map = True
        self.frontiers = []
        self.visited = set()
        self.nav = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.distance_threshold = 0.2
        
        # Timer variables
        self.map_update_duration = 5.0  # Duration to wait for map update (in seconds)
        self.map_update_timer = self.create_timer(0.5, self.update_map_timer_callback)
        self.map_update_start_time = None
        self.ready_to_explore = False
        
    
    def map_callback(self, msg):
        self.get_logger().info("Occupancy Grid received")
        # Get all Occupancy grid's data as attributes
        self.map_data = msg
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y    
        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.grid = np.asarray(msg.data).reshape((self.map_height, self.map_width))
        
        # Reset map update timer each time a new map is received
        if not self.map_update_start_time:
            self.map_update_start_time = self.get_clock().now()
    
    
    def update_map_timer_callback(self):
        """Timer callback to handle map updates."""
        if self.map_update_start_time is not None:
            elapsed_time = (self.get_clock().now() - self.map_update_start_time).nanoseconds / 1e9

            if elapsed_time >= self.map_update_duration:
                self.ready_to_explore = True
                self.get_logger().info("Map updated for 5 seconds. Finding frontiers...")
                self.find_and_explore_frontiers()
                self.map_update_start_time = None  # Reset the timer
                self.ready_to_explore = False  # Reset readiness to explore
    
    
    def find_and_explore_frontiers(self):
        if not self.ready_to_explore:
            return

        self.frontiers = self.find_frontiers()
        if self.frontiers:
            self.explore_frontiers()
        else:
            self.get_logger().info("No frontiers found! Exploration completed")
            raise SystemExit
    
    
    def find_frontiers(self):
        frontiers = []
        for i in range(1, self.map_height - 1):
            for j in range(1, self.map_width - 1):
                if self.grid[i, j] == -1:  # Unknown cell
                    # Check if any neighboring cell is free space (0) within 20cm
                    cell_range = round(0.1 / self.resolution)
                    if np.any(self.grid[i-cell_range:i+cell_range, j-cell_range:j+cell_range] == 0):
                        # frontier point = (y, x)
                        frontier_point = (i * self.resolution + self.origin_y,
                                          j * self.resolution + self.origin_x)
                        if self.is_valid_frontier(frontier_point) == True:
                            frontiers.append(frontier_point)

        self.get_logger().info(f'Found {len(frontiers)} frontiers')
        return frontiers
    
    
    def is_valid_frontier(self, frontier):
        # Convert back to Grid cell
        y = floor((frontier[0] - self.origin_y) / self.resolution)
        x = floor((frontier[1] - self.origin_x) / self.resolution)
        
        if self.is_already_visited(frontier) == True:
            return False
        
        # Make sure it isn't near any wall
        cell_range = round(0.1 / self.resolution)
        if np.any(self.grid[y-cell_range:y+cell_range, x-cell_range:x+cell_range] == 100):
            return False
        
        # Valid
        return True
    
    
    def is_already_visited(self, frontier):
        for visited in self.visited:
            if self.calculate_distance(visited, frontier) < self.distance_threshold / self.resolution:
                return True
        return False


    def calculate_distance(self, pointA, pointB):
        return np.sqrt(((pointA[0] - pointB[0]) ** 2) + ((pointA[1] - pointB[1]) ** 2))


    def explore_frontiers(self):
        # Select first frontier
        frontier = self.frontiers[0]
        self.navigate_to_frontier(frontier)
        
        
    def navigate_to_frontier(self, frontier):
        try:
            # Create a target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.pose.position.x = frontier[1]
            target_pose.pose.position.y = frontier[0]
            target_pose.pose.orientation.w = 1.0  # Facing forward

            self.nav.goToPose(target_pose)

            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1200.0): # type: ignore
                    self.get_logger().warn("Timeout occurred, cancelling goal")
                    self.nav.cancelTask()

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
                self.visited.add(frontier)
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Navigation canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().error('Navigation failed!')

        except Exception as e:
            self.get_logger().error(f'Err: {str(e)}')
            
    
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
             

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
