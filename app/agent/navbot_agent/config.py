"""Central constants for the navbot agent. Everything an operator app or a
mode definition might need to tweak lives here, not scattered in modules."""

import os

WS_HOST = "0.0.0.0"
WS_PORT = 8080

# UDP fast path (teleop/telemetry/video), GCS-style. Same port number as the
# WS — UDP and TCP are separate namespaces, one number to open on firewalls.
# A session falls back to WS whenever no client datagram arrived recently.
UDP_ALIVE_S = 3.0

# Command limits — must match config/drive_lut.yaml max_cmd_v and the
# local_planner's w_max. The agent clamps every teleop command to these.
V_MAX = 0.40        # m/s
W_MAX = 1.2         # rad/s

TELEOP_RATE_HZ = 20.0     # /cmd_vel publish rate while teleop is fresh
TELEOP_STALE_S = 0.4      # no teleop msg for this long -> one zero, then silence

TELEMETRY_HZ = 2.0
ATTITUDE_HZ = 10.0        # roll/pitch/yaw stream for the console instruments

# IMU mounting offset, subtracted from the displayed attitude (display
# convention: roll + = right, pitch + = nose up). Measured 2026-07-07 with
# the robot flat on the bench; re-measure (set to 0, read ATT via ws_probe)
# if the MPU6050 is ever remounted.
ATT_TRIM_ROLL_DEG = -5.1
ATT_TRIM_PITCH_DEG = 15.1
LOG_RING = 200            # launch/agent log lines replayed to a new client

# Workspace + environment for ros2 launch subprocesses.
WS_ROOT = os.path.expanduser("~sunrise/rdk-x5-navbot") \
    if os.path.isdir(os.path.expanduser("~sunrise/rdk-x5-navbot")) \
    else "/home/sunrise/rdk-x5-navbot"
ROS_SETUP = "/opt/tros/humble/setup.bash"
WS_SETUP = os.path.join(WS_ROOT, "install/setup.bash")

# Set by --motors-off: forces motors:=false in every mode (safe dev/demo).
FORCE_MOTORS_OFF = False

# mode name -> (launch file, motors). stopped = no launch process.
# mapping = teleop + SLAM build; navigate = localization + goal navigation.
MODES = {
    "stopped":  None,
    "observe":  ("navigation.launch.py", False),
    "manual":   ("manual.launch.py", True),
    "auto":     ("navigation.launch.py", True),
    "mapping":  ("mapping.launch.py", True),
    "navigate": ("autonav.launch.py", True),
}

# Modes in which the operator's teleop drives /cmd_vel (manual + mapping).
# In navigate the goal_navigator drives; observe/auto have their own planner.
TELEOP_MODES = ("manual", "mapping")

LAUNCH_PKG = "navbot_bringup"
LAUNCH_PIDFILE = "/run/navbot-agent.launch.pid"
LAUNCH_STOP_SIGINT_S = 12.0   # grace after SIGINT before SIGTERM
LAUNCH_STOP_SIGTERM_S = 3.0   # grace after SIGTERM before SIGKILL
MODE_ACTIVE_TIMEOUT_S = 25.0  # /obstacles (or /estop service) must appear by then

# Operator-togglable models/features. obstacle_avoidance gates safety_gate's
# sector (RPLidar scan_sectors) forward-block and defaults ON; the perception
# overlays (yolo11, depthanything) default OFF — opt-in, cost BPU/bandwidth.
MODELS = ("obstacle_avoidance", "yolo11", "depthanything")
MODEL_ENABLE_TOPIC = {
    "obstacle_avoidance": "/perception/obstacle_avoidance_enable",
    "yolo11": "/perception/yolo11_enable",
    "depthanything": "/perception/depth_enable",
}
MODEL_DEFAULTS = {"obstacle_avoidance": True, "yolo11": False, "depthanything": False}

# Cameras: name -> binary-protocol id. Front is CompressedImage (JPEG),
# sides are raw YUYV 320x240 that the agent JPEG-encodes.
CAMERAS = {"front": 0, "left": 1, "right": 2}
VIDEO_DEFAULT_FPS = {"front": 15.0, "left": 5.0, "right": 5.0}
VIDEO_SD_SIZE = (640, 360)    # front "sd" re-encode size
VIDEO_JPEG_QUALITY = 70

# SLAM map view (console MAP toggle). Low rate — the map itself only
# changes a few times a second at most, and it's opt-in per session.
MAP_PUSH_HZ = 1.0

# Where saved maps live and the default basename the console "Save Map"
# writes to (and autonav.launch.py loads for localization). save_map writes
# both <base>.pgm/.yaml (viewable) and <base>.posegraph/.data (loadable).
MAP_DIR = os.path.join(WS_ROOT, "maps")
DEFAULT_MAP = "current"        # basename under MAP_DIR

# Topics whose publish rate the telemetry reports.
RATE_TOPICS = [
    "/cam_front/image_raw",
    "/cam_left/image_raw",
    "/cam_right/image_raw",
    "/obstacles",
    "/cmd_vel",
    "/cmd_vel_safe",
    "/range_forward",
    "/imu/data",
]
