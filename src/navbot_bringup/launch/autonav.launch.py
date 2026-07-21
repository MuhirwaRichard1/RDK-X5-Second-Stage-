# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
autonav.launch.py — console-driven autonomous goal navigation. The reactive
stack with the goal_navigator in place of local_planner, PLUS lidar SLAM in
localization mode (loads a saved map, relocalizes, and recovers from a kidnap).
This is the agent's "navigate" mode.

    cameras + RPLidar C1 + imu_driver + scan_sectors + safety_gate
      + motor_controller (motors:=true) + detection/depth BPU (idle)
    + lidar_slam.launch.py (slam_mode:=none, odom_source:=icp):
        icp_odometry (odom->base_link) + base_link->laser TF
    + amcl_localization.launch.py (map_file:=<map>): map_server serves the
        saved map, nav2_amcl publishes map->odom (globally localizable)
    + goal_navigator: /goal + /obstacles + map->base_link -> /cmd_vel

Operator flow: map a room in MAPPING mode, SAVE MAP, then switch to NAVIGATE
and pick the map. The robot does not know where it is in that map yet, so it
scatters AMCL's particles over the whole thing and wanders until they collapse
(state LOCALIZING) — then click a point on the console map and it drives there,
avoiding obstacles. A click made while it is still localizing is queued. Lift
and carry it -> it scatters again, re-finds itself, and resumes the same goal.

    ros2 launch navbot_bringup autonav.launch.py                    # dry
    ros2 launch navbot_bringup autonav.launch.py motors:=true       # drives!
    ros2 launch navbot_bringup autonav.launch.py map_file:=maps/foo # other map

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
    map_file = DeclareLaunchArgument(
        "map_file",
        default_value="/home/sunrise/rdk-x5-navbot/maps/current",
        description="saved pose-graph basename to localize against "
                    "(what MAPPING's SAVE MAP writes)")

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_cameras"),
                         "launch/three_cam.launch.py")))

    # Odometry only (icp_odometry -> odom->base_link, + base_link->laser TF).
    # slam_mode:=none — AMCL below owns map->odom, not slam_toolbox.
    # run_lidar:=false — the sllidar_node below owns the serial port.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_slam"),
                         "launch/lidar_slam.launch.py")),
        launch_arguments={"slam_mode": "none",
                          "odom_source": "icp",
                          "run_lidar": "false"}.items())

    # Localization against the saved map: map_server + nav2_amcl. Unlike
    # slam_toolbox localization this can GLOBALLY localize — goal_navigator
    # kicks /reinitialize_global_localization and spins until it converges.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navbot_slam"),
                         "launch/amcl_localization.launch.py")),
        launch_arguments={"map_file": LaunchConfiguration("map_file")}.items())

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
        slam, localization,
    ])
