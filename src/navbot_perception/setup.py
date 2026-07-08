from setuptools import setup

package_name = "navbot_perception"

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
    description="On-BPU perception + obstacle fusion for RDK X5.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "obstacle_fusion = navbot_perception.obstacle_fusion:main",
            "detection_bpu = navbot_perception.detection_bpu:main",
            "depth_bpu = navbot_perception.depth_bpu:main",
            "depth_freespace = navbot_perception.depth_freespace:main",
        ],
    },
)
