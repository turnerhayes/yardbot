import os
from glob import glob
from setuptools import setup

package_name = "yardbot_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "urdf"),
            glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="turner",
    maintainer_email="turner@todo.todo",
    description="URDF and description files for Yardbot",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [],
    },
)
