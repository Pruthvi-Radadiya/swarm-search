import os
from glob import glob

from setuptools import find_packages, setup

package_name = "map_merge_swarm"

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
    maintainer_email="pruth@todo.todo",
    description="Occupancy map merging for multi-robot swarm search",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "merge_node = map_merge_swarm.merge_node:main",
            "gt_map_tf_node = map_merge_swarm.gt_map_tf_node:main",
            "shared_map_relay = map_merge_swarm.shared_map_relay:main",
        ],
    },
)
