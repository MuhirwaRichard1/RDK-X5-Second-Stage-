# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
lidar_slam.launch.py — Track B lidar SLAM: RPLidar C1 + slam_toolbox.

Pipeline:
  RPLidar C1 (/dev/ttyUSB0 @ 460800) ─► sllidar_node ─► /scan ───────────┐
  /cmd_vel_safe + /imu/data ─► dr_odom ─► /odom_dr + TF odom->base_link ─┤
                                                                         ▼
                       slam_toolbox (async mapping) ─► /map + map->odom + closures

Like slam.launch.py (Track A, retired), this does NOT start the sensors —
the agent's active modes (manual/observe/auto) run the lidar driver, IMU and
cameras. It subscribes to raw topics, so it works identically on the LIVE
robot and on a replayed rosbag.

  live mapping : agent in an active mode, then
                 ros2 launch navbot_slam lidar_slam.launch.py
  standalone   : agent stopped -> add run_lidar:=true (starts sllidar here;
                 double-starting it against an active mode fails on the port)
                 and run imu_driver by hand for dr_odom
  bag replay   : ros2 launch navbot_slam lidar_slam.launch.py use_sim_time:=true
                 + ros2 bag play <bag> --clock --rate 0.5
  record for offline build: /scan /imu/data /cmd_vel_safe /cmd_vel

Save a finished map:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: /home/sunrise/maps/arena}}"

TF: base_link -> laser. The C1 is mounted with its laser-frame 0 deg facing
the robot's REAR (established live 2026-07-13: front/back obstacles were
swapped), so lidar_yaw defaults to pi. Keep in sync with the 180 deg
yaw_offset_deg that manual/navigation launches pass to scan_sectors and
safety_gate. x/z are still rough estimates — measure and override.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_lidar = LaunchConfiguration("run_lidar")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a bag with --clock"),
        DeclareLaunchArgument("run_lidar", default_value="false",
                              description="start the sllidar driver here — "
                                          "only when the agent is stopped (its "
                                          "active modes already run it)"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        # rough base_link -> laser mount — MEASURE & override
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.14159265"),
    ]

    st = {"use_sim_time": use_sim_time}

    # --- RPLidar C1 driver -> /scan (live only: a bag supplies /scan when
    # use_sim_time, and a wall-clock driver would corrupt sim-time SLAM) -----
    lidar = Node(
        package="sllidar_ros2", executable="sllidar_node", name="sllidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": LaunchConfiguration("serial_port"),
            "serial_baudrate": 460800,          # C1 fixed rate
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",            # 5 kHz, 10 Hz, 720 pts
        }],
        output="screen",
        condition=IfCondition(PythonExpression(
            ["'", run_lidar, "' == 'true' and '", use_sim_time, "' != 'true'"])))

    # --- dead-reckoning odometry backbone: /odom_dr + TF odom->base_link ----
    dr_odom = Node(
        package="navbot_drive", executable="dr_odom", name="dr_odom",
        parameters=[st], output="screen")

    # --- static extrinsics (rough — see docstring) ---------------------------
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["--x", LaunchConfiguration("lidar_x"), "--y", "0.0",
                   "--z", LaunchConfiguration("lidar_z"),
                   "--roll", "0", "--pitch", "0",
                   "--yaw", LaunchConfiguration("lidar_yaw"),
                   "--frame-id", "base_link", "--child-frame-id", "laser"],
        parameters=[st])

    # --- slam_toolbox: async mapper -> /map + map->odom ----------------------
    slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            os.path.join(get_package_share_directory("navbot_slam"),
                         "config", "slam_toolbox.yaml"),
            st],
        output="screen")

    return LaunchDescription(args + [lidar, dr_odom, tf_laser, slam])
