#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
from tf2_ros import Buffer, TransformListener
import numpy as np

from sklearn.cluster import DBSCAN
from scipy.spatial import distance
class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node') # type: ignore
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.init_map = True
        self.frontiers = []
        self.visited = []
        self.nav = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    
    def map_callback(self, msg):
        # if self.init_map:
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_theta = msg.info.origin.orientation.w
        self.init_map = False
            
        # print(f"Map origin: {self.origin_x}, {self.origin_y}")
        
        # Get all Occupancy grid's data as attributes
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        # print(f"Map dimensions: {self.map_height} cols x {self.map_width} rows")
        # Map grid is is row-major order, starting with (0, 0)
        self.grid = np.asarray(msg.data).reshape((self.map_height, self.map_width))
        
        if self.nav.isTaskComplete():
            self.frontiers = self.get_frontiers()
            
            # Sanity check: no frontiers found
            if self.frontiers is not None:
                frontier = self.select_frontier(self.frontiers)
                self.cluster_frontiers()
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
                    if -1 in self.grid[x-1:x+2, y-1:y+2] and (x, y) not in self.visited:
                        frontiers.append((x, y))
        self.get_logger().info(f'Found {len(frontiers)} frontiers')
        return frontiers


    def cluster_frontiers(self):
            if not self.frontiers:
                return

            # Convert to numpy array for clustering
            frontier_points = np.array(self.frontiers)

            # Use DBSCAN clustering to group nearby frontiers
            clustering = DBSCAN(eps=3, min_samples=2).fit(frontier_points)
            labels = clustering.labels_

            # Calculate the centroid of each cluster
            unique_labels = set(labels)
            clustered_frontiers = []
            for label in unique_labels:
                if label == -1:
                    continue  # Ignore noise points
                label_indices = np.where(labels == label)
                cluster_points = frontier_points[label_indices]
                centroid = np.mean(cluster_points, axis=0)
                clustered_frontiers.append(centroid)

            self.frontiers = [tuple(map(int, point)) for point in clustered_frontiers]
            self.get_logger().info(f'Clustered to {len(self.frontiers)} frontier centroids')

    def prioritize_frontiers(self, robot_position):
        # Prioritize frontiers based on distance from the robot
        prioritized_frontiers = sorted(self.frontiers, key=lambda f: distance.euclidean(robot_position, f))
        return prioritized_frontiers


    def select_frontier(self, frontiers):
        # Get the current robot position
        try:
            map_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_position = (
                int(map_transform.transform.translation.y / 0.05),  # Row index
                int(map_transform.transform.translation.x / 0.05)   # Column index
            )
        except Exception as e:
            self.get_logger().error(e)
            return

        prioritized_frontiers = self.prioritize_frontiers(robot_position)

        for frontier in prioritized_frontiers:
            if tuple(frontier) not in self.visited:
                self.go_to_frontier(frontier)
                self.visited.append(tuple(frontier))
                break  # Navigate to one frontier at a time
    
    
    def go_to_frontier(self, frontier):
        """Order the robot to traverse to a given frontier coordinate.

        Args:
            frontier (float x, float y): A coordinate of float values
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = frontier[0] * self.resolution + self.origin_x
        goal_pose.pose.position.y = frontier[1] * self.resolution + self.origin_y
        goal_pose.pose.orientation.w = 1.0  # Facing forward
        self.nav.goToPose(goal_pose)
    
    
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
             

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
