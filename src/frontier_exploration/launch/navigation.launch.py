import os
from struct import pack
from sys import executable

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    print(
        os.path.join(
            os.getcwd(),
            "src",
            "frontier_exploration",
            "param",
            "explore_param.yaml",
        )
    )

    nav_params = os.path.join(
        os.getcwd(),
        "src",
        "frontier_exploration",
        "param",
        "explore_param.yaml",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
                ),
                launch_arguments={"params_file": nav_params}.items(),
            ),
        ]
    )
