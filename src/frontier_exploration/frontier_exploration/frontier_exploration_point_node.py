#!/usr/bin/env python3

from copy import deepcopy
from math import floor, sqrt
import math
from tkinter.tix import Tree
import frontier_exploration
import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
import numpy as np
from rclpy.duration import Duration


class FrontierExplorationGoalNode(Node):
    """Frontier-based Exploration ROS 2 node for Humble Hawksbill.

    This ROS 2 node takes data from the robot's Occupancy Grid
    which is used to locate and navigate frontiers to reach a manually selected goal.

    Attributes:
        map_sub: A subsciber to the Occupancy Grid with an assigned callback.
        frontiers: A list containing the frontiers after an Occupancy Grid update.
        visited: A set containing all visited frontiers.
        point_sub: A subscriber to the '/clicked_point' topic from rivz.
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
        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )

        self.frontiers = []
        self.visited = set()
        self.failed = set()  # Set to hold all failed navigations
        self.nav = BasicNavigator()
        self.distance_threshold = 0.2
        self.last_goal_x = None
        self.last_goal_y = None

        # Timer variables
        # Duration to wait for map update (in seconds)
        self.map_update_duration = 5.0
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

    def point_callback(self, msg):
        """Callback function from the Publish Point subscriber from RViz2

        This callback function stores the clicked point from RViz2 and is used
        as the end goal of the exploration.

        Args:
            msg (Message): Data about the Published Point
        """
        self.get_logger().info("Exploration Goal Point received")
        self.point_data = msg
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y

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
                # TODO: Improve try algorithm
                self.try_reach_goal()

                # Check to see if robot has reached desired goal
                # Use robot's last destination as position
                if self.last_goal_x is not None:
                    if self.goal_distance((self.last_goal_y, self.last_goal_x)) < (
                        0.1 / self.resolution
                    ):
                        self.get_logger().info("Goal reached!")
                        self.last_goal_x = None
                        self.last_goal_y = None
                        raise SystemExit

                self.map_update_start_time = None  # Reset the timer
                self.ready_to_explore = False  # Reset readiness to explore

    def try_reach_goal(self):
        try:
            # Convert back to Grid cell
            y = floor((self.goal_y - self.origin_y) / self.resolution)
            x = floor((self.goal_x - self.origin_x) / self.resolution)

            # Goal is unknown still
            if self.grid[y, x] == -1:
                self.find_and_explore_frontiers()
            # Goal is known, unoccupied space
            elif self.grid[y, x] == 0:
                self.navigate_to_location((self.goal_y, self.goal_x))

        except IndexError:
            self.get_logger().warn(
                "Location not currently in Occupancy Grid, keep exploring"
            )
            self.find_and_explore_frontiers()
        except AttributeError:
            # Goal not been selected yet
            pass

    def find_and_explore_frontiers(self):
        """Contoller function for the Node's exploration logic.

        Raises:
            SystemExit: Stop spinning the node when all frontiers are found.
        """
        if not self.ready_to_explore:
            return

        self.frontiers = self.find_frontiers()
        if self.frontiers:
            frontier = self.select_frontier()
            self.navigate_to_location(frontier)
        else:
            # Check failed frontiers to see if they can be re-explored
            self.frontiers = self.try_failed_frontiers()
            if self.frontiers:
                self.get_logger().info(
                    "No frontiers left, trying previously failed frontiers"
                )
                frontier = self.select_frontier()
                self.navigate_to_location(frontier)
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
                            unknowns = self.count_unknown_neighbours(j, i)
                            frontiers.append([frontier_point, unknowns])

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
                # self.get_logger().warn("Frontier near too many obstacles")
                return False
            else:
                return True
        except KeyError:
            # Valid
            return True

    def is_already_visited(self, frontier):
        """Determines if a frontier has already been visited.

        A frontier is classed as visited if it's withing a certain proximity of
        other previously visited or failed frontiers.

        Args:
            frontier (tuple(int, int)): Frontier coordinate (y, x)

        Returns:
            Boolean: Indicator if the frontier has already been visited
        """
        for visited in self.visited:
            if self.calculate_distance(visited, frontier) < self.distance_threshold:
                return True

        return self.visited_failed(frontier)

    def calculate_distance(self, pointA, pointB):
        """Calculates the distance (in metres) between two coordinates.

        Args:
            pointA (tuple(int, int)): Coordinate of the first point.
            pointB (tuple(int, int)): Coordinate of the second point.

        Returns:
            ndarray: The Euclidean distance between the two points.
        """
        return np.sqrt(((pointA[0] - pointB[0]) ** 2) + ((pointA[1] - pointB[1]) ** 2))

    def visited_failed(self, frontier):
        """Determine if a failed frontier has previously been visited

        Args:
            frontier (list[y, x]): coordinate of frontier

        Returns:
            boolean: Indicates whether it has already been visited
        """
        for failed in self.failed:
            if self.calculate_distance(failed, frontier) < self.distance_threshold:
                return True
        return False

    def count_unknown_neighbours(self, x, y):
        """Counts the number of unknown neighbouring points around a given coordinate.

        Args:
            x (int): X coordinate
            y (int): Y coordinate

        Returns:
            int: number of neighbouring unknown points
        """
        check_region = self.grid[y - 2 : y + 2, x - 2 : x + 2]
        unique, counts = np.unique(check_region, return_counts=True)
        occurrences = dict(zip(unique, counts))
        # If no unknowns, set occurrence to 0
        try:
            return occurrences[-1]
        except KeyError:
            return 0

    def try_failed_frontiers(self):
        """See if any of the failed frontiers are still unknown and try to navigate there.

        Returns:
            list: list([frontier, neighbour occurrences])
        """
        frontiers = []
        # Check if any are still unknown
        for failed in self.failed:
            grid_x = floor((failed[1] - self.origin_x) / self.resolution)
            grid_y = floor((failed[0] - self.origin_y) / self.resolution)
            if self.grid[grid_y, grid_x] == -1:
                unknown = self.count_unknown_neighbours(grid_x, grid_y)
                frontiers.append([failed, unknown])
                self.failed.remove(failed)

        self.get_logger().info(f"Found {len(frontiers)} failed frontiers")
        return frontiers

    def goal_distance(self, frontier):
        """Calculate the distance between a given frontier and the goal.

        Args:
            frontier (tuple): A given coordinate of a frontier (y, x)
        """
        f_x = frontier[1]
        f_y = frontier[0]

        return np.sqrt(((f_x - self.goal_x) ** 2) + ((f_y - self.goal_y) ** 2))

    def select_frontier(self):
        """Selection function for a frontier

        Returns:
            tuple(int, int): Coordinate of frontier
        """
        # Naive approach: first frontier
        # frontier = self.frontiers[0][0]

        # # Approach 2: Select frontier based on number of neighbouring unknowns
        # sorted_frontiers = sorted(self.frontiers, reverse=True, key=lambda f: f[1])

        # Approach 3: Select frontier based on how close it is to the goal
        sorted_frontiers = sorted(
            self.frontiers, key=lambda f: self.goal_distance(f[0])
        )

        frontier = sorted_frontiers[0][0]
        return frontier

    def navigate_to_location(self, frontier):
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
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):  # type: ignore
                    self.get_logger().warn("Timeout occurred, cancelling goal")
                    self.nav.cancelTask()

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Navigation succeeded!")
                prev_visted_len = len(self.visited)
                # Only save robot position if it was a success
                self.last_goal_x = frontier[1]
                self.last_goal_y = frontier[0]
                self.visited.add(frontier)
                if len(self.visited) == prev_visted_len:
                    self.get_logger().warn("Visited set not incremented")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Navigation canceled!")
            elif result == TaskResult.FAILED:
                self.get_logger().error("Navigation failed!")
                self.failed.add(frontier)

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
    node = FrontierExplorationGoalNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
