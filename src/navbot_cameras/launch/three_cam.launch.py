# Copyright (c) 2026 Ricardo Muhirwa — MIT License
"""
three_cam.launch.py — bring up the 3 USB cameras of the RDK X5 Tri-Cam NavBot.

Topology: all 3 cameras hang off a single RDK X5 USB 3.0 port via a 4-port
USB 3.0 hub. The cameras themselves are UVC **USB 2.0** devices (HBVCAM OV2710
front + 2 wide-angle sides), so they share the host's single 480 Mbps High-Speed
bus. MJPEG is therefore mandatory, and the side cameras run at reduced
resolution/framerate to stay inside the bandwidth budget (see PROPOSAL Risk R2).

Each camera runs as its own `hobot_usb_cam` node in a dedicated namespace, so it
publishes:  /<ns>/image_raw  and  /<ns>/camera_info
  cam_front : 1280x720 @ 30 fps  (detection, depth, visual odometry)
  cam_left  :  640x480 @ 15 fps  (left surround free-space)
  cam_right :  640x480 @ 15 fps  (right surround free-space)

Device paths: USB enumeration order (/dev/video0,2,4...) is NOT stable across
reboots/replug. Pass stable /dev/v4l/by-path/* symlinks instead — find them with:
    ls -l /dev/v4l/by-path/
and override the *_device launch args below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _cam(ns, device, width, height, fps):
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
            "pixel_format": "mjpeg",   # compressed on the wire — required (USB-2 bus)
            "io_method": "mmap",
            "zero_copy": False,
        }],
        remappings=[
            ("image", "image_raw"),
            ("camera_info", "camera_info"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )


def generate_launch_description():
    # Defaults are by-path placeholders — EDIT to match `ls -l /dev/v4l/by-path/`.
    front_dev = DeclareLaunchArgument(
        "front_device", default_value="/dev/video0",
        description="front cam device (prefer /dev/v4l/by-path/*-index0)")
    left_dev = DeclareLaunchArgument(
        "left_device", default_value="/dev/video2",
        description="left cam device")
    right_dev = DeclareLaunchArgument(
        "right_device", default_value="/dev/video4",
        description="right cam device")

    # hobot_shm enables zero-copy/shared-mem transport (launch once for all cams)
    shm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("hobot_shm"),
                         "launch/hobot_shm.launch.py")))

    return LaunchDescription([
        front_dev, left_dev, right_dev,
        shm,
        _cam("cam_front", LaunchConfiguration("front_device"), 1280, 720, 30),
        _cam("cam_left", LaunchConfiguration("left_device"), 640, 480, 15),
        _cam("cam_right", LaunchConfiguration("right_device"), 640, 480, 15),
    ])
