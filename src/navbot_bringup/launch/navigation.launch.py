# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
navigation.launch.py — PIDNet-based reactive navigation stack.

    cameras (three_cam.launch.py)
        -> obstacle_fusion  (PIDNet on BPU, 3 cams -> /obstacles)
        -> local_planner    (/obstacles -> /cmd_vel)
        -> safety_gate      (/cmd_vel + TF-Luna + /estop -> /cmd_vel_safe)
        -> motor_controller (only when motors:=true)

Default is motors:=false so the whole perception/planning chain can be
verified with `ros2 topic hz|echo` before anything moves:

    ros2 launch navbot_bringup navigation.launch.py               # dry
    ros2 launch navbot_bringup navigation.launch.py motors:=true  # drives!

E-stop at any time:  ros2 service call /estop std_srvs/srv/SetBool "{data: true}"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    motors = DeclareLaunchArgument(
        "motors", default_value="false",
        description="true = motor_controller drives the wheels")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_cameras"),
                         "launch/three_cam.launch.py")))

    return LaunchDescription([
        motors,
        cameras,
        Node(package="navbot_slam", executable="imu_driver",
             name="imu_driver", output="screen"),
        Node(package="navbot_perception", executable="obstacle_fusion",
             name="obstacle_fusion", output="screen"),
        Node(package="navbot_navigation", executable="local_planner",
             name="local_planner", output="screen"),
        Node(package="navbot_drive", executable="safety_gate",
             name="safety_gate", output="screen"),
        Node(package="navbot_drive", executable="motor_controller",
             name="motor_controller", output="screen",
             condition=IfCondition(LaunchConfiguration("motors"))),
    ])
