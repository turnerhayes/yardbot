import os
from glob import glob
from setuptools import setup

package_name = "yardbot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yardbot",
    maintainer_email="todo@todo.com",
    description="Bringup launch files and nodes for Yardbot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "depth_to_grid = yardbot_bringup.depth_to_grid_node:main",
            "qos_relay     = yardbot_bringup.qos_relay_node:main",
            "apriltag_overlay = yardbot_bringup.apriltag_overlay_node:main",
            "sabertooth_node = yardbot_bringup.sabertooth_node:main",
        ],
    },
)
