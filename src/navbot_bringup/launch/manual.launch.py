# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
manual.launch.py — teleop stack: like navigation.launch.py but WITHOUT
local_planner, so an operator (navbot_agent teleop) is the only /cmd_vel
publisher. scan_sectors stays up for the operator's sector HUD, and every
command still passes through safety_gate (scan forward clamp + /estop).

    cameras (three_cam.launch.py)   — operator video only, no obstacle role
    RPLidar C1 (sllidar_node) -> /scan
        -> scan_sectors     (/scan -> /obstacles sector classification;
                             PRIMARY obstacle sensor — replaced the BPU
                             camera pipelines, costs no BPU, works in the dark)
    operator /cmd_vel
        -> safety_gate      (/cmd_vel + /scan + /obstacles + /estop ->
                             /cmd_vel_safe; assist:=true here = 60 cm
                             proximity ring stop + gentle steer toward the
                             clearer side when something is within 0.8 m
                             ahead — manual drives get obstacle avoidance,
                             not just obstacle stops)
        -> motor_controller (only when motors:=true)
        -> detection_bpu    (YOLO11 on BPU, idle until /perception/yolo11_enable)
        -> depth_bpu        (Depth Anything on BPU, front cam only, idle
                              until /perception/depth_enable — keeps the
                              console depth overlay now that depth_freespace
                              is gone)

    ros2 launch navbot_bringup manual.launch.py               # dry
    ros2 launch navbot_bringup manual.launch.py motors:=true  # drives!

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
        # MOUNT: the C1 sits with its laser-frame 0° facing the robot's REAR
        # (found live 2026-07-13: front/back swapped) — 180° maps laser rays
        # to robot bearings. Keep in sync with lidar_slam.launch.py lidar_yaw.
        Node(package="navbot_perception", executable="scan_sectors",
             name="scan_sectors", output="screen",
             parameters=[{"yaw_offset_deg": 180.0}]),
        Node(package="navbot_drive", executable="safety_gate",
             name="safety_gate", output="screen",
             parameters=[{"assist": True, "yaw_offset_deg": 180.0}]),
        Node(package="navbot_drive", executable="motor_controller",
             name="motor_controller", output="screen",
             condition=IfCondition(LaunchConfiguration("motors"))),
        Node(package="navbot_perception", executable="detection_bpu",
             name="detection_bpu", output="screen"),
        Node(package="navbot_perception", executable="depth_bpu",
             name="depth_bpu", output="screen"),
    ])
