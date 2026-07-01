import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bugbot_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BugBotLab",
    maintainer_email="jerome.a.graves@gmail.com",
    description="ROS 2 bridge and learned controller for the BugBot X-omni holonomic robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist_bridge = bugbot_ros.twist_bridge_node:main",
            "policy_node = bugbot_ros.policy_node:main",
        ],
    },
)
