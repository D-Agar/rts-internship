from setuptools import find_packages, setup
import os
import glob

package_name = "frontier_exploration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob.glob(os.path.join("worlds", "*.world"), recursive=True),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob.glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daniel",
    maintainer_email="daniel@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explore = frontier_exploration.frontier_exploration_node:main",
            "assisted_explore = frontier_exploration.frontier_exploration_assisted_node:main",
        ],
    },
)
