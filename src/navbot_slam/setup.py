from setuptools import setup

package_name = "navbot_slam"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "smbus2"],
    zip_safe=True,
    maintainer="Ricardo Muhirwa",
    maintainer_email="muhirwaricardo12@gmail.com",
    description="Visual-inertial SLAM, mapping, relocalization, and the MPU6050 IMU driver for RDK X5.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_driver = navbot_slam.imu_driver:main",
            # vio_slam and relocalizer to be added (see README / ROADMAP W4-W5)
        ],
    },
)
