import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    frontier_rviz_config = os.path.join(
        get_package_share_directory("frontier_exploration"),
        "config",
        "explore_config.rviz",
    )

    exploration = Node(
        package="frontier_exploration",
        executable="explore",
        name="exploration",
    )

    explore_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + frontier_rviz_config],
    )

    return LaunchDescription([explore_rviz, exploration])
