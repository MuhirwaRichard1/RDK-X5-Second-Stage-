# RDK X5 Tri-Cam NavBot 🤖

> Indoor robot that **maps, navigates to a clicked goal, and recovers from being kidnapped** —
> with **no wheel encoders** — on the D-Robotics **RDK X5**.
> Odometry is measured against the world by **laser scan-matching**; the BPU runs
> **Depth Anything V2 + YOLO11m** concurrently for depth and semantic overlays.

| | |
|---|---|
| **Brain** | RDK X5 — 8× Cortex-A55, ~10 TOPS Bayes-e BPU, ROS 2 Humble + TROS |
| **Odometry** | RPLidar C1 → `icp_odometry` scan-matching, fused with MPU6050 gyro via `robot_localization` EKF |
| **SLAM** | `slam_toolbox` — async mapping, then localization against the saved map |
| **Cameras** | Front: HBVCAM **OV2710 100°** · Sides: 2× wide-angle USB · all on **one** USB 3.0 port via hub |
| **IMU** | **MPU6050** (I2C5 @ 0x68) — yaw rate at ~190 Hz between the 10 Hz scans |
| **Drive** | 2× DC geared motors (**no encoder**) via **L298N**, duty↔velocity LUT + SLAM correction |
| **Range** | TF-Luna (I2C5 @ 0x10) — independent forward safety |
| **BPU** | Depth Anything V2 ViT-S (392) ~2.8 FPS · YOLO11m ~52 ms/frame — two workloads, one BPU |

## 🎬 Demo

| | |
|---|---|
| **Stage 3 demo** | https://youtu.be/Z3CnLehWs7o |
| Extended demo — operator console POV | https://youtu.be/6Rp8K-f7oq8 |
| Live AI inference on the BPU | https://youtu.be/p5Sa7evwUvI |

---

## 🚀 Quick Start

**Robot:** RDK X5 with TROS (ROS 2 Humble) flashed, wired per [`docs/bom.md`](docs/bom.md).
**PC:** Windows or Linux, for the operator console.

```bash
# ── 1. Clone (the RPLidar driver is third-party and gitignored — clone it in) ──
git clone https://github.com/MuhirwaRichard1/RDK-X5-Second-Stage-.git ~/rdk-x5-navbot
cd ~/rdk-x5-navbot
git clone https://github.com/Slamtec/sllidar_ros2 src/sllidar_ros2

# ── 2. Dependencies ──
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-robot-localization \
                    ros-humble-rtabmap-odom
# The BPU stack (hobot_dnn) needs numpy 1.x — DO NOT let anything pull numpy 2.
sudo pip3 install "numpy==1.26.4" "opencv-python-headless<4.11" smbus2

# ── 3. Build ──
source /opt/tros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash        # required in EVERY shell (custom navbot_msgs)

# ── 4. Launch — robot side ──
sudo ./app/agent/run_agent.sh    # or: sudo systemctl start navbot-agent

# ── 5. Launch — PC side ──
cd app/desktop
pip install -r requirements.txt
python -m navbot_console          # connect to ws://<robot-ip>:8080
```

**One-time board config** (already applied on this unit — redo on a fresh board):

- `dtoverlay=dtoverlay_pwm3` in `/boot/config.txt` (motor PWM on pins 32/33) + reboot
- `options uvcvideo quirks=128` in `/etc/modprobe.d/uvcvideo-navbot.conf`
  (3 UVC cameras on one USB-2 bus)
- BPU models: Depth Anything build in `~/Desktop/RDK/model_output_vits392/`,
  YOLO11m in `~/rdk_model_zoo/samples/vision/ultralytics_yolo/model/`
  (conversion procedure: [`docs/depth_anything_conversion.md`](docs/depth_anything_conversion.md))
- Cameras resolve by physical port (`/dev/v4l/by-path/...`): **front = port 1.1**
  (1 MP, MJPEG), sides = ports 1.3 / 1.4 (YUYV 320×240 — MJPEG on the side cams
  saturates the bus)

**Seed a map** so `navigate` works before you've mapped anything:

```bash
cp maps/arena_20260713.posegraph maps/current.posegraph
cp maps/arena_20260713.data      maps/current.data
```

Full walkthrough — map → save → navigate → kidnap recovery:
**[NEXT_STEPS.md](NEXT_STEPS.md)**.

---

## 🎮 Running the mission

The desktop console owns the whole loop. Modes are exclusive buttons (the agent
runs exactly **one** launch at a time):
`stopped · observe · manual · auto · mapping · navigate`.

- **MAPPING** — teleop-drive while `slam_toolbox` builds the map live in the MAP
  view; hit **SAVE MAP**.
- **NAVIGATE** — `slam_toolbox` localization loads the saved map; **click the map
  to set a goal** and `goal_navigator` drives there with reactive avoidance under
  an always-on safety gate. A **lift is detected → RELOCALIZE** (kidnap recovery)
  → resume.

Console features: live camera feeds with the obstacle-sector HUD, joystick/WASD
teleop, attitude and health panels, E-stop. Protocol and safety model:
[`docs/operator_app.md`](docs/operator_app.md).

### Headless / ROS-only

Motors are **OFF by default** in every launch — verify topics before arming.

```bash
source install/setup.bash
ros2 launch navbot_bringup navigation.launch.py     # or mapping / autonav / manual

ros2 topic hz /obstacles           # scan_sectors, 24 sectors, ~10 Hz
ros2 topic echo /cmd_vel_safe      # what the motors would receive

# arm the motors
ros2 launch navbot_bringup navigation.launch.py motors:=true

# kill switch (also: unplug the L298N)
ros2 service call /estop std_srvs/srv/SetBool "{data: true}"
```

**Chain:** `sllidar_node` → **`scan_sectors`** (`/scan` → 24 sectors on
`/obstacles`, `navbot_msgs/Sectors`; unknown sector = **blocked**) →
**`goal_navigator`** (or `local_planner` in reactive-only mode) → `/cmd_vel` →
**`safety_gate`** (lidar return under `stop_cm` or `/estop` blocks forward) →
**`motor_controller`** (0.2 s dead-man; `motors:=true` only).

`depth_bpu` and `detection_bpu` run alongside, each gated by its own latch topic
(`/perception/depth_enable`, `/perception/yolo11_enable`) so they can be toggled
live without restarting the stack.

**Tuning:** sector width and ranges in `scan_sectors` (`sector_deg`,
`block_range_m`, `yaw_offset_deg`); gap acceptance in `goal_navigator` /
`local_planner` (`min_run_deg`, `front_cone_deg`); stop distance in `safety_gate`
(`stop_cm`).

### Standalone camera-only demos (no ROS)

Earlier single-purpose avoidance demos, kept because they're the cleanest way to
benchmark a single model. All serve a live view at `http://<robot-ip>:8080`.
Always run **`--dry-run` first** (perception only, motors off); the TF-Luna is a
hard forward stop in every variant.

| Script | Perception | Loop |
|---|---|---|
| `scripts/07_pidnet_avoid.py` | PIDNet-S floor segmentation (9 ms/frame) | ~15 Hz |
| `scripts/08_yolo_avoid.py` | YOLO11m detection (52 ms/frame), widest-gap steering | ~10 Hz |
| `scripts/09_hybrid_avoid.py` | PIDNet floor **and** YOLO object veto, fused per column | ~7.5 Hz |
| `scripts/10_depth_preview.py` | Depth Anything V2 preview (`--model vits392\|vitsopt2`) | 2.9 / 1.2 FPS |

```bash
sudo python3 scripts/09_hybrid_avoid.py --dry-run     # watch :8080, then drop --dry-run
```

---

## 🛡️ Safety

Three independent layers, in order of trust:

1. **`safety_gate`** — deliberately dumb node between planner and motors. Blocks
   forward motion on a lidar return closer than `stop_cm` (30 cm) in the forward
   half-width, on `/estop`, or on input timeout. Does not depend on the AI stack
   being correct.
2. **`motor_controller` dead-man** — no `/cmd_vel_safe` within `cmd_timeout`
   (0.2 s) and the motors coast to a stop. Last line even if everything upstream
   hangs.
3. **Fail-safe sectors** — `scan_sectors` marks any sector with no usable rays
   `UNKNOWN`, and every consumer treats UNKNOWN as **blocked**. Conservative when
   blind, never optimistic.

E-stop is available as a ROS service, as a console button, and over the console's
UDP fast path so it never queues behind video.

---

## 📄 Documentation

- **[NEXT_STEPS.md](NEXT_STEPS.md)** — build & run from zero, full mission walkthrough
- **[PROPOSAL.md](PROPOSAL.md)** — Stage 2 submission: concept, architecture, engineering plan
- **[ROADMAP.md](ROADMAP.md)** — week-by-week milestones with exit criteria
- **[docs/architecture.md](docs/architecture.md)** — design decisions & ADRs
- **[docs/bom.md](docs/bom.md)** — bill of materials + power architecture
- **[docs/depth_anything_conversion.md](docs/depth_anything_conversion.md)** — Depth Anything → BPU `.bin`
- **[docs/operator_app.md](docs/operator_app.md)** — console + agent protocol, safety model
- **[docs/vio_slam_plan.md](docs/vio_slam_plan.md)** — original VIO plan (superseded, kept for context)

## Repo layout

```
README.md  PROPOSAL.md  ROADMAP.md  NEXT_STEPS.md
docs/          # architecture, BOM, model conversion, operator app
src/           # ROS 2 packages (colcon): bringup, msgs, cameras, perception, slam, navigation, drive
app/           # operator app: agent/ (robot WebSocket+UDP agent) + desktop/ (PySide6 console)
config/        # YAML params, camera calib, duty<->velocity LUT
scripts/       # standalone demos, calibration, bench analysis
bench_tests/   # hardware bring-up tests (I2C scan, motor direction, safety gate)
maps/          # saved slam_toolbox maps
stl_files/     # printable mounts (lidar, camera, battery holder)
Camera_calibration/   # checkerboard captures + intrinsics
```

> `launch/` and `models/` are placeholders. Launch files live in each package
> under `src/*/launch/`; compiled BPU `.bin` models are not committed (size) —
> see the Quick Start for where they're expected on disk.

---

## Why it's not a stock demo

Most RDK demos use one camera, **wheel encoders**, and never lose their pose.
This robot has **no encoders** — translation is measured against the world by
scan-to-map ICP rather than counted at the wheel, so battery sag, slip and carpet
don't smear the map. It also explicitly solves the **kidnapped-robot problem**:
lift it, move it, and a lift/scan-jump detector drops it into RELOCALIZE, rotates
until localization re-converges, and resumes the mission.

The Stage 2 plan had Depth Anything as the *primary* mapping sensor via VIO. In
practice ViT-S 518 couldn't get a contiguous allocation from the stock 320 MB ION
pool even with the BPU idle, and ViT-S 392 tops out near 2.8 Hz — too slow to key
RGB-D odometry. The RPLidar C1 took over odometry and obstacle sectors; the BPU
models stayed on as depth and semantic overlays. See
[`docs/architecture.md`](docs/architecture.md).

## License

MIT — see [LICENSE](LICENSE).
