# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
three_cam.launch.py — bring up the 3 USB cameras of the RDK X5 Tri-Cam NavBot.

Topology (verified 2026-07-06): the cameras plug directly into the X5's USB-A
ports, which all sit behind the board's onboard Genesys hub on a single
480 Mbps USB-2 bus. Bandwidth is the binding constraint, and the two side
cameras (1e45:8022 clones) request the bus MAXIMUM for any MJPEG mode — even
320x240 — so only ONE of them can stream MJPEG at a time. The working recipe:

  * uvcvideo must be loaded with quirks=128 (FIX_BANDWIDTH; persisted in
    /etc/modprobe.d/uvcvideo-navbot.conf). The quirk only helps UNCOMPRESSED
    formats on kernel 6.1, hence YUYV for the sides.
  * cam_front : 0bdc:8088 (1 MP)  MJPEG 1280x720 @ 30 fps — honest bandwidth
  * cam_left  : 1e45:8022         YUYV   320x240 @ 30 fps (delivers ~16-29)
  * cam_right : 1e45:8022         YUYV   320x240 @ 30 fps (delivers ~16-29)
  All three measured streaming simultaneously at these settings; the clones
  only advertise 30 fps (no 15 fps modes), so framerate stays 30 everywhere.

Each camera runs as its own `hobot_usb_cam` node in a dedicated namespace, so it
publishes:  /<ns>/image_raw  and  /<ns>/camera_info

Device paths: /dev/videoN enumeration order is NOT stable across reboots or
replugging. Defaults below use /dev/v4l/by-path/* symlinks, which are fixed per
PHYSICAL PORT (front = port 1.1, the one the 0bdc:8088 is in). If a camera
moves to a different port, re-check with:  ls -l /dev/v4l/by-path/
Left/right assignment is by port (1.3 = left, 1.4 = right) — verify against the
actual mounting and swap the launch args if mirrored.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

_BY_PATH = "/dev/v4l/by-path/platform-xhci-hcd.2.auto-usb-0:{port}:1.0-video-index0"


def _cam(ns, device, width, height, fps, pixel_format, calib_file=""):
    """One hobot_usb_cam node, namespaced, with image->image_raw remap."""
    return Node(
        package="hobot_usb_cam",
        executable="hobot_usb_cam",
        name="hobot_usb_cam",
        namespace=ns,
        parameters=[{
            "frame_id": ns,
            "video_device": device,
            "image_width": width,
            "image_height": height,
            "framerate": fps,
            "pixel_format": pixel_format,
            "io_method": "mmap",
            "zero_copy": False,
            "camera_calibration_file_path": calib_file,
        }],
        remappings=[
            ("image", "image_raw"),
            ("camera_info", "camera_info"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )


def generate_launch_description():
    front_dev = DeclareLaunchArgument(
        "front_device", default_value=_BY_PATH.format(port="1.1"),
        description="front cam (0bdc:8088 1MP) device path")
    left_dev = DeclareLaunchArgument(
        "left_device", default_value=_BY_PATH.format(port="1.3"),
        description="left cam (1e45:8022) device path")
    right_dev = DeclareLaunchArgument(
        "right_device", default_value=_BY_PATH.format(port="1.4"),
        description="right cam (1e45:8022) device path")

    # hobot_shm enables zero-copy/shared-mem transport (launch once for all cams)
    shm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("hobot_shm"),
                         "launch/hobot_shm.launch.py")))

    # intrinsics from scripts/11_front_cam_calib.py; empty until calibrated
    front_calib = "/home/sunrise/rdk-x5-navbot/config/camera_front.yaml"
    if not os.path.exists(front_calib):
        front_calib = ""

    return LaunchDescription([
        front_dev, left_dev, right_dev,
        shm,
        _cam("cam_front", LaunchConfiguration("front_device"),
             1280, 720, 30, "mjpeg", front_calib),
        _cam("cam_left", LaunchConfiguration("left_device"),
             320, 240, 30, "yuyv"),
        _cam("cam_right", LaunchConfiguration("right_device"),
             320, 240, 30, "yuyv"),
    ])
