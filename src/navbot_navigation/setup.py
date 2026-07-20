from setuptools import setup

package_name = "navbot_navigation"

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
    description="Reactive local planner + behaviour management for RDK X5.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "local_planner = navbot_navigation.local_planner:main",
            "goal_navigator = navbot_navigation.goal_navigator:main",
        ],
    },
)
