# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
navigation.launch.py — lidar-based reactive navigation stack.

    cameras (three_cam.launch.py)   — operator video only, no obstacle role
    RPLidar C1 (sllidar_node) -> /scan
        -> scan_sectors     (/scan -> /obstacles sector classification;
                             replaced the PIDNet/Depth-Anything camera
                             pipelines — no BPU cost, works in the dark)
        -> local_planner    (/obstacles -> /cmd_vel)
        -> safety_gate      (/cmd_vel + /scan + /estop -> /cmd_vel_safe)
        -> motor_controller (only when motors:=true)
        -> detection_bpu    (YOLO11 on BPU, idle until /perception/yolo11_enable)
        -> depth_bpu        (Depth Anything on BPU, front cam only, idle
                              until /perception/depth_enable)

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
        Node(package="sllidar_ros2", executable="sllidar_node",
             name="sllidar_node", output="screen",
             parameters=[{"channel_type": "serial",
                          "serial_port": "/dev/ttyUSB0",
                          "serial_baudrate": 460800,    # C1 fixed rate
                          "frame_id": "laser",
                          "inverted": False,
                          "angle_compensate": True,
                          "scan_mode": "Standard"}]),
        Node(package="navbot_perception", executable="scan_sectors",
             name="scan_sectors", output="screen"),
        Node(package="navbot_navigation", executable="local_planner",
             name="local_planner", output="screen"),
        Node(package="navbot_drive", executable="safety_gate",
             name="safety_gate", output="screen"),
        Node(package="navbot_drive", executable="motor_controller",
             name="motor_controller", output="screen",
             condition=IfCondition(LaunchConfiguration("motors"))),
        Node(package="navbot_perception", executable="detection_bpu",
             name="detection_bpu", output="screen"),
        Node(package="navbot_perception", executable="depth_bpu",
             name="depth_bpu", output="screen"),
    ])
