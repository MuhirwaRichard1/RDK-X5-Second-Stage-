import os
from glob import glob

from setuptools import setup

package_name = "navbot_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricardo Muhirwa",
    maintainer_email="muhirwaricardo12@gmail.com",
    description="Top-level launch files for the RDK X5 Tri-Cam NavBot.",
    license="MIT",
)
