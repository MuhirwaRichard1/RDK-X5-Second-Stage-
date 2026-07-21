# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
autonav.launch.py — console-driven autonomous goal navigation. The reactive
stack with the goal_navigator in place of local_planner, PLUS live lidar SLAM
(slam_toolbox mapping mode). This is the agent's "navigate" mode.

    cameras + RPLidar C1 + imu_driver + scan_sectors + safety_gate
      + motor_controller (motors:=true) + detection/depth BPU (idle)
    + lidar_slam.launch.py (slam_mode:=mapping, odom_source:=icp):
        icp_odometry (odom->base_link) + base_link->laser TF
        + slam_toolbox building a live map -> /map + map->odom
    + goal_navigator: /goal + /obstacles + map->base_link -> /cmd_vel

Operator flow: switch to NAVIGATE. slam_toolbox starts a fresh map from the
robot's current position (so map->odom is valid at once — no cold-start pose
problem), and the map grows as the robot drives into new space. Click a point
on the console map and it drives there, avoiding obstacles. SAVE MAP persists
the grown map. A lift/bump triggers a brief re-match spin (small bumps only —
there is no global relocalization; a real lift-and-carry can corrupt the map).

    ros2 launch navbot_bringup autonav.launch.py                    # dry
    ros2 launch navbot_bringup autonav.launch.py motors:=true       # drives!

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
    # Accepted (the agent appends map_file:= for navigate) but UNUSED: NAVIGATE
    # builds a fresh live map rather than loading a saved one. Kept declared so
    # the launch does not reject the argument.
    map_file = DeclareLaunchArgument(
        "map_file",
        default_value="/home/sunrise/rdk-x5-navbot/maps/current",
        description="unused — NAVIGATE builds a fresh map (kept for arg compat)")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_cameras"),
                         "launch/three_cam.launch.py")))

    # Live lidar SLAM: icp_odometry (odom->base_link) + slam_toolbox mapping
    # (builds /map and publishes map->odom). run_lidar:=false — the sllidar_node
    # below owns the serial port.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_slam"),
                         "launch/lidar_slam.launch.py")),
        launch_arguments={"slam_mode": "mapping",
                          "odom_source": "icp",
                          "run_lidar": "false"}.items())

    return LaunchDescription([
        motors, map_file,
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
        # MOUNT: C1 laser-frame 0° faces the robot's REAR — 180° offset (keep in
        # sync with lidar_slam.launch.py lidar_yaw=pi and the other launches).
        Node(package="navbot_perception", executable="scan_sectors",
             name="scan_sectors", output="screen",
             parameters=[{"yaw_offset_deg": 180.0}]),
        Node(package="navbot_navigation", executable="goal_navigator",
             name="goal_navigator", output="screen"),
        Node(package="navbot_drive", executable="safety_gate",
             name="safety_gate", output="screen",
             parameters=[{"yaw_offset_deg": 180.0}]),
        Node(package="navbot_drive", executable="motor_controller",
             name="motor_controller", output="screen",
             condition=IfCondition(LaunchConfiguration("motors"))),
        Node(package="navbot_perception", executable="detection_bpu",
             name="detection_bpu", output="screen"),
        Node(package="navbot_perception", executable="depth_bpu",
             name="depth_bpu", output="screen"),
        slam,
    ])
