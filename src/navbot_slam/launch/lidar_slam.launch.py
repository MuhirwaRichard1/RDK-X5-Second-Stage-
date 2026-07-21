# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
lidar_slam.launch.py — Track B lidar SLAM: RPLidar C1 + slam_toolbox.

Pipeline:
  RPLidar C1 (/dev/ttyUSB0 @ 460800) ─► sllidar_node ─► /scan ───────────┐
  <odometry source, selectable> ─► TF odom->base_link ───────────────────┤
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
                 and run imu_driver by hand for dr_odom / fused
  bag replay   : ros2 launch navbot_slam lidar_slam.launch.py use_sim_time:=true
                 + ros2 bag play <bag> --clock --rate 0.5
  record for offline build: /scan /imu/data /cmd_vel_safe /cmd_vel

ODOMETRY SOURCE — odom_source:= (the encoderless-drift fix). A/B on bag
mapping_fresh_20260713 (2026-07-21) mapped icp visibly crisper than dr (which
sprays/fragments) — icp is now the DEFAULT. fused came out WORSE (doubled
walls) — the EKF needs tuning before use; prefer icp until then.
  icp   (default) LASER scan-matching odometry (rtabmap icp_odometry): x/y/yaw
                  measured against the walls. The core fix for map smearing.
  dr              dead-reckoning fallback: /cmd_vel_safe (open-loop translation)
                  + IMU gyro yaw. Survives feature-poor views icp can't; drifts.
  fused           icp translation + IMU gyro yaw via a robot_localization EKF
                  (needs tuning — see A/B note above).

SLAM MODE — slam_mode:= (what slam_toolbox does with the scans):
  mapping (default) build a fresh map (async node) -> /map + map->odom.
  localization      load map_file:=<basename> and relocalize against it
                    (localization node). NOTE: this only scan-matches LOCALLY
                    from the map origin — it cannot find the robot in the map.
                    NAVIGATE uses nav2_amcl instead (amcl_localization.launch.py).
  none              odometry + TF only; something else owns map->odom.
  The bringup modes drive these: mapping.launch.py -> mapping,
  autonav.launch.py -> none (+ amcl_localization.launch.py).
Exactly one node owns the odom->base_link TF in every mode; slam_toolbox always
consumes that TF + /scan, so the three are drop-in interchangeable.

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
    odom_source = LaunchConfiguration("odom_source")
    slam_mode = LaunchConfiguration("slam_mode")
    map_file = LaunchConfiguration("map_file")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a bag with --clock"),
        DeclareLaunchArgument("run_lidar", default_value="false",
                              description="start the sllidar driver here — "
                                          "only when the agent is stopped (its "
                                          "active modes already run it)"),
        DeclareLaunchArgument("odom_source", default_value="icp",
                              description="odom->base_link source: "
                                          "icp | dr | fused (see docstring). "
                                          "icp (laser scan-matching) is the "
                                          "default — A/B 2026-07-21 showed it "
                                          "maps far crisper than dead-reckoning."),
        DeclareLaunchArgument("slam_mode", default_value="mapping",
                              description="mapping = build a new map (async node); "
                                          "localization = load map_file and "
                                          "scan-match against it (local only — "
                                          "NAVIGATE uses amcl instead); "
                                          "none = odometry + TF only."),
        DeclareLaunchArgument(
            "map_file",
            default_value="/home/sunrise/rdk-x5-navbot/maps/arena_20260713",
            description="pose-graph basename (no extension) loaded in "
                        "slam_mode:=localization"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        # rough base_link -> laser mount — MEASURE & override
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.14159265"),
    ]

    st = {"use_sim_time": use_sim_time}
    cfg = get_package_share_directory("navbot_slam")

    # odom_source == <val> as a launch condition
    def when(val):
        return IfCondition(PythonExpression(
            ["'", odom_source, "' == '", val, "'"]))

    # slam_mode == <val> as a launch condition
    def when_slam(val):
        return IfCondition(PythonExpression(
            ["'", slam_mode, "' == '", val, "'"]))

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

    # ============ ODOMETRY SOURCES (exactly one publishes odom->base_link) ===

    # (dr) dead-reckoning backbone: /odom_dr + TF odom->base_link -------------
    dr_odom = Node(
        package="navbot_drive", executable="dr_odom", name="dr_odom",
        parameters=[st], output="screen", condition=when("dr"))

    icp_cfg = os.path.join(cfg, "config", "icp_odometry.yaml")

    # (icp) laser scan-matching odometry, publishing the TF directly ----------
    icp_tf = Node(
        package="rtabmap_odom", executable="icp_odometry", name="icp_odometry",
        parameters=[icp_cfg, {"publish_tf": True}, st],
        remappings=[("scan", "/scan")],
        output="screen", condition=when("icp"))

    # (fused) icp for translation, EKF fuses in the gyro yaw and owns the TF --
    icp_topic = Node(
        package="rtabmap_odom", executable="icp_odometry", name="icp_odometry",
        parameters=[icp_cfg, {"publish_tf": False}, st],
        remappings=[("scan", "/scan"), ("odom", "/odom_icp")],
        output="screen", condition=when("fused"))

    tf_imu = Node(   # base_link -> imu_link (axis-aligned mount) for the EKF
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_imu",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        parameters=[st], condition=when("fused"))

    ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(cfg, "config", "ekf.yaml"), st],
        output="screen", condition=when("fused"))

    # --- slam_toolbox (mapping): async mapper -> /map + map->odom ------------
    slam_map = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[os.path.join(cfg, "config", "slam_toolbox.yaml"), st],
        output="screen", condition=when_slam("mapping"))

    # --- slam_toolbox (localization): load map_file, relocalize -> map->odom -
    # (also the kidnap relocalizer). map_file_name is injected here from the
    # map_file arg so one config serves any saved map.
    slam_loc = Node(
        package="slam_toolbox", executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[os.path.join(cfg, "config", "slam_toolbox_localization.yaml"),
                    {"map_file_name": map_file}, st],
        output="screen", condition=when_slam("localization"))

    return LaunchDescription(args + [
        lidar, tf_laser,
        dr_odom, icp_tf, icp_topic, tf_imu, ekf,
        slam_map, slam_loc])
