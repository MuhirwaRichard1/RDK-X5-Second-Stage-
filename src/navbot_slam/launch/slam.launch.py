# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
slam.launch.py — Track A visual SLAM: RTAB-Map RGB-D + IMU for the RDK X5.

Pipeline:
  /cam_front/image_raw (JPEG) + /range_forward (TF-Luna) ─► slam_rgbd ─►
    rectified RGB + registered METRIC depth  ─┐
  /imu/data ─► imu_filter_madgwick ─► /imu/data_filtered (orientation) ─┤
                                                                        ▼
                              rtabmap_odom/rgbd_odometry ─► /odom + odom->base_link
                              rtabmap_slam/rtabmap       ─► /map + map->odom + loop closure

This launch runs the SLAM back-end ONLY — it does NOT start the cameras/IMU
(the agent's active mode publishes those) and it does NOT start detection_bpu/
depth_freespace (they contend for the BPU with slam_rgbd's depth model). It
subscribes to raw topics, so it works identically on the LIVE robot and on a
replayed rosbag (set use_sim_time:=true for bag playback with --clock).

TF: rtabmap needs base_link -> cam_front (camera optical frame) and
base_link -> imu_link. The defaults below are ROUGH mount estimates — measure
the real lever arm/orientation (vio_slam_plan Phase 1.3) and override via args.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a bag with --clock"),
        # rough base_link -> cam_front (optical) mount — MEASURE & override
        DeclareLaunchArgument("cam_x", default_value="0.10"),
        DeclareLaunchArgument("cam_z", default_value="0.15"),
    ]

    st = {"use_sim_time": use_sim_time}

    # --- SLAM RGB-D front-end (decode + rectify + metric depth) --------------
    slam_rgbd = Node(
        package="navbot_perception", executable="slam_rgbd", name="slam_rgbd",
        parameters=[st], output="screen")

    # --- IMU orientation filter: /imu/data -> /imu/data_filtered ------------
    madgwick = Node(
        package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[{"use_mag": False, "publish_tf": False,
                     "world_frame": "enu", **st}],
        remappings=[("imu/data_raw", "/imu/data"),
                    ("imu/data", "/imu/data_filtered")])

    # --- static extrinsics (rough — see docstring) --------------------------
    # base_link (x fwd, y left, z up) -> cam_front (optical: x right, y down,
    # z fwd): roll=-90 deg, yaw=-90 deg.
    tf_cam = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_cam_front",
        arguments=["--x", LaunchConfiguration("cam_x"), "--y", "0.0",
                   "--z", LaunchConfiguration("cam_z"),
                   "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
                   "--frame-id", "base_link", "--child-frame-id", "cam_front"],
        parameters=[st])
    tf_imu = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_imu",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.05",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        parameters=[st])

    # --- RTAB-Map (rgbd_odometry + rtabmap) via its packaged launch ---------
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rtabmap_launch"),
            "launch", "rtabmap.launch.py")),
        launch_arguments={
            "rgb_topic": "/slam_rgbd/rgb/image_rect",
            "depth_topic": "/slam_rgbd/depth/image_rect",
            "camera_info_topic": "/slam_rgbd/rgb/camera_info",
            "frame_id": "base_link",
            "imu_topic": "/imu/data_filtered",
            "wait_imu_to_init": "true",
            "approx_sync": "true",
            # slow (~3-6 Hz) depth => generous queues + a loose odom guess so
            # tracking survives between depth frames (gyro prior from the IMU).
            "queue_size": "30",
            "wait_for_transform": "0.3",
            "qos": "1",                       # slam_rgbd/imu publish RELIABLE
            "rtabmap_viz": "false",
            "rviz": "false",
            "database_path": "/home/sunrise/.ros/rtabmap.db",
            "rtabmap_args": "--delete_db_on_start",
            "use_sim_time": use_sim_time,
        }.items())

    return LaunchDescription(args + [slam_rgbd, madgwick, tf_cam, tf_imu, rtabmap])
