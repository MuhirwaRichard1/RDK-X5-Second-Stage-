from setuptools import setup
from glob import glob

package_name = "navbot_cameras"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricardo Muhirwa",
    maintainer_email="muhirwaricardo12@gmail.com",
    description="3-camera USB bring-up for RDK X5 (wraps hobot_usb_cam).",
    license="MIT",
    entry_points={"console_scripts": []},
)
