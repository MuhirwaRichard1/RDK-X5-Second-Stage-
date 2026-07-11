from setuptools import setup

package_name = "navbot_drive"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricardo Muhirwa",
    maintainer_email="muhirwaricardo12@gmail.com",
    description="Open-loop L298N differential drive + safety gate (encoderless) for RDK X5.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motor_controller = navbot_drive.motor_controller:main",
            "safety_gate = navbot_drive.safety_gate:main",
            "dr_odom = navbot_drive.dr_odom:main",
        ],
    },
)
