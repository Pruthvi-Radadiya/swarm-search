import os
from glob import glob

from setuptools import find_packages, setup

package_name = "explore_swarm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pruth",
    maintainer_email="pruthviradadiya@gmail.com",
    description="Exploration for multi-robot swarm search",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "frontier_detector = explore_swarm.frontier_detector:main",
            "goal_assigner = explore_swarm.goal_assigner:main",
        ],
    },
)
