# Build & Run the NavBot from Zero

> How to bring the RDK X5 Tri-Cam NavBot up on a fresh checkout and run the full
> mission: **map → save → navigate to a goal → recover from a kidnap**. For the
> concept/architecture see [PROPOSAL.md](PROPOSAL.md); for the operator-app
> internals see [docs/operator_app.md](docs/operator_app.md).

The robot brain is a **D-Robotics RDK X5** running **TROS (ROS 2 Humble)**. The PC
side (operator console) runs on Windows or Linux.

---

## 1. Prerequisites (on the robot)

Assumes TROS Humble is already flashed on the X5 and the hardware is wired per
[docs/bom.md](docs/bom.md): 3 USB cameras, MPU6050 IMU on I2C5, L298N + 2 motors,
and an **RPLidar C1** on `/dev/ttyUSB0`.

One-time board config (already applied on this unit — redo on a fresh board):
- `dtoverlay=dtoverlay_pwm3` in `/boot/config.txt` (motor PWM on pins 32/33) + reboot
- `options uvcvideo quirks=128` in `/etc/modprobe.d/uvcvideo-navbot.conf`
  (3 UVC cameras on one USB-2 bus)

System / Python dependencies:
```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-robot-localization \
                    ros-humble-rtabmap-odom
# BPU stack (hobot_dnn) needs numpy 1.x — DO NOT let anything pull numpy 2.
sudo pip3 install "numpy==1.26.4" "opencv-python-headless<4.11" smbus2
```

## 2. Get the code

```bash
git clone git@github.com:MuhirwaRichard1/RDK-X5-Second-Stage-.git ~/rdk-x5-navbot
cd ~/rdk-x5-navbot

# The RPLidar driver is vendored but gitignored (third-party) — reclone it:
git clone https://github.com/Slamtec/sllidar_ros2 src/sllidar_ros2
```

## 3. Build

```bash
source /opt/tros/humble/setup.bash          # ROS 2 Humble + TROS
colcon build --symlink-install
source install/setup.bash                   # required in EVERY shell (custom msgs)
```

## 4. Seed a default map (so NAVIGATE works before you map anything)

`navigate` mode loads `maps/current.*` (what MAPPING's **SAVE MAP** writes). A
committed demo map ships as `maps/arena_20260713.*` — copy it to bootstrap:
```bash
cp maps/arena_20260713.posegraph maps/current.posegraph
cp maps/arena_20260713.data      maps/current.data
```
(Or skip this and just map your own room first — step 6.)

---

## 5. Run — the operator console (normal path)

**Robot side** — start the agent (owns the mode launches, WS+UDP on :8080):
```bash
sudo ./app/agent/run_agent.sh          # or: sudo systemctl start navbot-agent
```

**PC side** — the desktop console:
```bash
cd app/desktop
pip install -r requirements.txt
python -m navbot_console                # connect to ws://<robot-ip>:8080
```

Modes are exclusive buttons in the console (the agent runs exactly ONE launch
at a time): `stopped · observe · manual · auto · mapping · navigate`.

## 6. Run — the full mission from the console

1. **MAPPING** — click MAPPING (motors on). Drive with the joystick/WASD; open the
   **MAP** view and watch the occupancy grid build. Drive the whole area, close
   loops (return past where you started). Keep the speed slider ≥ 30 %.
2. **SAVE MAP** — click it. Writes `maps/current.{pgm,yaml,posegraph,data}` on the
   robot (the log confirms `map ... saved`).
3. **NAVIGATE** — click NAVIGATE (motors on). It loads `maps/current` and localizes
   (the robot marker snaps onto the map after a short drive).
4. **Set a goal** — click a point on the MAP. A green cross marks it; the robot
   plans + drives there, avoiding obstacles (log shows `state -> NAVIGATE/AVOID/
   ARRIVED`). It stops within ~15 cm of the goal.
5. **Kidnap** — lift and carry the robot elsewhere mid-run. It detects the lift,
   enters `RELOCALIZE` (spins slowly to re-match the map), then resumes to the
   goal. **E-stop** is always live (button, or the physical latch).

## 7. Run — from a terminal (equivalent, for debugging)

The console just drives these; you can run them directly when the agent is
**stopped** (never next to a live mode — see traps):
```bash
# build/verify a map (drive it with teleop from another tool)
ros2 launch navbot_bringup mapping.launch.py motors:=true
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /home/sunrise/rdk-x5-navbot/maps/current}}"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: /home/sunrise/rdk-x5-navbot/maps/current}"

# localize + navigate against a saved map
ros2 launch navbot_bringup autonav.launch.py motors:=true map_file:=maps/current
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}}}"

# SLAM only (odometry A/B, offline map builds)
ros2 launch navbot_slam lidar_slam.launch.py            # odom_source:=icp, mapping
ros2 launch navbot_slam lidar_slam.launch.py slam_mode:=localization map_file:=maps/current
```

## 8. Tests & offline verification

```bash
# planner logic (no hardware, no motors) — 7 scenarios
python3 bench_tests/07_goal_navigator_test.py

# rebuild/verify SLAM offline from a recorded bag — ALWAYS on a spare domain so
# the replayed /cmd_vel_safe can't reach the live agent's motor path:
ROS_DOMAIN_ID=42 ros2 launch navbot_slam lidar_slam.launch.py use_sim_time:=true odom_source:=icp
ROS_DOMAIN_ID=42 ros2 bag play bags/mapping_fresh_20260713 --clock --rate 1.0
```

## 9. Known traps (each cost hours — don't rediscover them)

- Source **both** `/opt/tros/humble/setup.bash` **and** `install/setup.bash` in every shell.
- The agent owns **ONE** `ros2 launch`. **Never** start a mode launch by hand next to a
  live mode (camera abort, GPIO unexport kills the live motors, lidar serial-port
  collision). Apply rebuilds by switching mode `stopped → manual` in the console.
- **Bag replay must run on a separate `ROS_DOMAIN_ID`.** The agent subscribes
  `/cmd_vel_safe` on the default domain — replaying a bag on domain 0 can drive the
  motors.
- Console **speed slider scales linear velocity only** — mapping at < 20 % gives
  creep-forward with normal-speed turns and ruins scan matching.
- **C1 mount:** laser 0° faces the robot's REAR. If you remount it, update
  `yaw_offset_deg` (scan_sectors + safety_gate) **and** `lidar_yaw`/`odom` together.
- Keep the robot **still ~3 s** after a mode starts (IMU gyro bias calibration).
- **numpy must stay 1.x** — numpy 2 breaks hobot_dnn/BPU.
- Root-run tools leave root-owned files in `sunrise`'s repo — `chown -R sunrise:sunrise`
  back (including `.git` after a root `git commit`).

---

### Odometry note (why the map is crisp)
The robot has **no wheel encoders**. Odometry comes from **laser scan-matching**
(`rtabmap_odom icp_odometry`, `odom_source:=icp`) — measured against the walls, not
dead-reckoned from commanded velocity. This is the default; `dr` (dead-reckoning) and
`fused` (icp + gyro EKF) remain selectable on `lidar_slam.launch.py`.
