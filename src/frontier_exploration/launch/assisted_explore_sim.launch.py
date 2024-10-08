import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    frontier_rviz_config = os.path.join(
        get_package_share_directory("frontier_exploration"),
        "config",
        "explore_config.rviz",
    )

    nav2_bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    nav_params = os.path.join(
        get_package_share_directory("frontier_exploration"),
        "config",
        "nav2_params.yaml",
    )

    slam_toolbox_bringup = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch"
    )

    tb3_house_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, "turtlebot3_house.launch.py")
        )
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "navigation_launch.py")
        ),
        launch_arguments={"params_file": nav_params}.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_bringup, "online_async_launch.py")
        )
    )

    exploration = Node(
        package="frontier_exploration",
        executable="assisted_explore",
        name="exploration",
    )

    explore_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + frontier_rviz_config],
    )

    return LaunchDescription(
        [
            tb3_house_launch,
            nav2_bringup_launch,
            slam_toolbox_launch,
            explore_rviz,
            exploration,
        ]
    )
