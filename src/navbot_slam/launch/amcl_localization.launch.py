# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
amcl_localization.launch.py — localize against a saved map with nav2_amcl.

    map_server (serves maps/<name>.pgm+yaml -> /map, latched)
      + amcl   (particle filter: /scan + /map + odom->base_link -> map->odom)
      + lifecycle_manager (autostart: configures & activates both)

This replaces slam_toolbox's localization mode in NAVIGATE. slam_toolbox only
scan-matches locally from an assumed pose — with no map_start_pose it assumes
the robot is at the map origin and publishes an identity map->odom forever.
AMCL can globally localize: goal_navigator calls
/reinitialize_global_localization on startup, which scatters particles across
the map, then spins the robot in place until the filter converges.

Odometry is unchanged: icp_odometry (from lidar_slam.launch.py) still owns
odom->base_link; amcl only supplies the map->odom correction on top.

    ros2 launch navbot_slam amcl_localization.launch.py map_file:=<basename>

map_file is a basename WITHOUT extension (matching the rest of the stack, where
it named a pose-graph); "<map_file>.yaml" is what map_server actually loads.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map_file",
            default_value="/home/sunrise/rdk-x5-navbot/maps/current",
            description="saved map basename, no extension — map_server loads "
                        "<map_file>.yaml (and the .pgm it names)"),
    ]

    cfg = os.path.join(get_package_share_directory("navbot_slam"),
                       "config", "amcl.yaml")
    st = {"use_sim_time": use_sim_time}

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[cfg, st, {"yaml_filename": [map_file, ".yaml"]}],
        output="screen")

    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl",
        parameters=[cfg, st], output="screen")

    # both are lifecycle nodes — nothing happens until they are configured and
    # activated, which the manager does for us on startup.
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[st, {"autostart": True,
                         "node_names": ["map_server", "amcl"]}])

    return LaunchDescription(args + [map_server, amcl, lifecycle])
