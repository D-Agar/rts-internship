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
import numpy as np
from rclpy.duration import Duration


class FrontierExplorationNode(Node):
    """Frontier-based Exploration ROS 2 node for Humble Hawksbill.

    This ROS 2 node takes data from the robot's Occupancy Grid
    which is used to locate and navigate to frontiers in an unknown environment.

    Attributes:
        map_sub: A subsciber to the Occupancy Grid with an assigned callback.
        frontiers: A list containing the frontiers after an Occupancy Grid update.
        visited: A set containing all visited frontiers.
        nav: An instance of Navigation2's simple commander.
        distance_threshold: Threshold (metres) for close adjacent frontiers
        map_update_duration: Duration to wait for map update (seconds)
        map_update_timer: Timer which calls map updater callback
        ready_to_explore: Boolean determining if the robot can explore
    """

    def __init__(self):
        """Initalises the ROS 2 Node."""
        super().__init__("frontier_exploration_node")  # type: ignore
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.frontiers = []
        self.visited = set()
        self.nav = BasicNavigator()
        self.distance_threshold = 0.2

        # Timer variables
        self.map_update_duration = 5.0  # Duration to wait for map update (in seconds)
        self.map_update_timer = self.create_timer(0.5, self.update_map_timer_callback)
        self.map_update_start_time = None
        self.ready_to_explore = False

    def map_callback(self, msg):
        """Callback function from Occupancy Grid subscriber.

        This callback function sets all the maps metadata as variables,
        which can be accessed throughout the class.

        Args:
            msg (Message): Data about the Occupancy Grid
        """
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
            elapsed_time = (
                self.get_clock().now() - self.map_update_start_time
            ).nanoseconds / 1e9

            if elapsed_time >= self.map_update_duration:
                self.ready_to_explore = True
                self.get_logger().info(
                    "Map updated for 5 seconds. Finding frontiers..."
                )
                self.find_and_explore_frontiers()
                self.map_update_start_time = None  # Reset the timer
                self.ready_to_explore = False  # Reset readiness to explore

    def find_and_explore_frontiers(self):
        """Contoller function for the Node's exploration logic.

        Raises:
            SystemExit: Stop spinning the node when all frontiers are found.
        """
        if not self.ready_to_explore:
            return

        self.frontiers = self.find_frontiers()
        if self.frontiers:
            # Select first frontier
            frontier = self.select_frontier()
            self.navigate_to_frontier(frontier)
        else:
            self.get_logger().info("No frontiers found! Exploration completed")
            raise SystemExit

    def find_frontiers(self):
        """Function which selects all viable frontiers.

        Returns:
            int[(int, int)]: List of frontier coordinates.
        """
        frontiers = []
        for i in range(1, self.map_height - 1):
            for j in range(1, self.map_width - 1):
                if self.grid[i, j] == 0:  # Known cell
                    # Check if any neighboring cell is unknown (-1) within 10cm
                    cell_range = round(0.1 / self.resolution)
                    if np.any(
                        self.grid[
                            i - cell_range : i + cell_range,
                            j - cell_range : j + cell_range,
                        ]
                        == -1
                    ):
                        # frontier point = (y, x)
                        frontier_point = (
                            i * self.resolution + self.origin_y,
                            j * self.resolution + self.origin_x,
                        )
                        if self.is_valid_frontier(frontier_point) == True:
                            # Add [frontier, unknown_cell_count] to frontiers
                            # for better frontier selection
                            # If no unknowns, set occurrence to 0
                            check_region = self.grid[i - 2 : i + 2, j - 2 : j + 2]
                            unique, counts = np.unique(check_region, return_counts=True)
                            occurrences = dict(zip(unique, counts))
                            try:
                                frontiers.append([frontier_point, occurrences[-1]])
                            except KeyError:
                                frontiers.append([frontier_point, 0])

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def is_valid_frontier(self, frontier):
        """Function which determines if a possible frontier is valid.

        Args:
            frontier (tuple(int, int)): Tuple of frontier coordinate (y, x)

        Returns:
            Boolean: Frontier validity indicator
        """
        # Convert back to Grid cell
        y = floor((frontier[0] - self.origin_y) / self.resolution)
        x = floor((frontier[1] - self.origin_x) / self.resolution)

        if self.is_already_visited(frontier) == True:
            return False

        # Make sure it isn't near any wall (15cm as it's roughly robot's radius)
        cell_range = round(0.15 / self.resolution)
        if np.any(
            self.grid[y - cell_range : y + cell_range, x - cell_range : x + cell_range]
            == 100
        ):
            return False

        check_region = self.grid[y - 2 : y + 2, x - 2 : x + 2]
        unique, counts = np.unique(check_region, return_counts=True)
        occurrences = dict(zip(unique, counts))

        try:
            if occurrences[100] > 2:
                self.get_logger().warn("Frontier near too many obstacles")
                return False
            else:
                return True
        except KeyError:
            # Valid
            return True

    def is_already_visited(self, frontier):
        """Determines if a frontier has already been visited.

        A frontier is classed as visited if it's withing a certain proximity of
        other previously visited frontiers.

        Args:
            frontier (tuple(int, int)): Frontier coordinate (y, x)

        Returns:
            Boolean: Indicator if the frontier has already been visited
        """
        for visited in self.visited:
            if self.calculate_distance(visited, frontier) < self.distance_threshold:
                return True
        return False

    def calculate_distance(self, pointA, pointB):
        """Calculates the distance (in metres) between two coordinates.

        Args:
            pointA (tuple(int, int)): Coordinate of the first point.
            pointB (tuple(int, int)): Coordinate of the second point.

        Returns:
            ndarray: The Euclidean distance between the two points.
        """
        return np.sqrt(((pointA[0] - pointB[0]) ** 2) + ((pointA[1] - pointB[1]) ** 2))

    def select_frontier(self):
        """Selection function for a frontier

        Returns:
            tuple(int, int): Coordinate of frontier
        """
        # Naive approach: first frontier
        # frontier = self.frontiers[0][0]

        # # Approach 2: Select frontier based on number of neighbouring unknowns
        sorted_frontiers = sorted(self.frontiers, reverse=True, key=lambda f: f[1])
        frontier = sorted_frontiers[0][0]

        return frontier

    def navigate_to_frontier(self, frontier):
        """Order the robot to navigate to a given frontier.

        Args:
            frontier (tuple(int, int)): Selected frontier coordinate
        """
        try:
            # Create a target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = frontier[1]
            target_pose.pose.position.y = frontier[0]
            target_pose.pose.orientation.w = 1.0  # Facing forward

            self.nav.goToPose(target_pose)

            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1200.0):  # type: ignore
                    self.get_logger().warn("Timeout occurred, cancelling goal")
                    self.nav.cancelTask()

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Navigation succeeded!")
                prev_visted_len = len(self.visited)
                self.visited.add(frontier)
                if len(self.visited) == prev_visted_len:
                    self.get_logger().warn("Visited set not incremented")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Navigation canceled!")
            elif result == TaskResult.FAILED:
                self.get_logger().error("Navigation failed!")

        except Exception as e:
            self.get_logger().error(f"Err: {str(e)}")

    def shutdown(self):
        """Method to shutdown the node."""
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Main function for the script's logic

    Args:
        args (List[str], optional): Args to be passed to the node. Defaults to None.
    """
    rclpy.init(args=args)
    node = FrontierExplorationNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
