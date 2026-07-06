# RDK X5 Tri-Cam NavBot 🤖

> Indoor robot that **maps, navigates to a saved goal, and recovers from being kidnapped** — using
> **3 USB cameras + an IMU, no wheel encoders** — on the D-Robotics **RDK X5**.
> BPU runs **Depth Anything + YOLO11**; the 8× A55 CPU runs **visual-inertial SLAM**, planning, and the
> open-loop motor loop.

| | |
|---|---|
| **Brain** | RDK X5 — 8× Cortex-A55, ~10 TOPS Bayes-e BPU, ROS 2 Humble + TROS |
| **Cameras** | Front: HBVCAM **OV2710 100°** · Sides: 2× wide-angle USB · all on **one** USB 3.0 port via hub |
| **IMU** | **MPU6050** (I2C) → fused with cameras for VIO SLAM + kidnap recovery |
| **Drive** | 2× DC geared motors (no encoder) via **L298N**, open-loop + SLAM correction |
| **Range** | TF-Luna LiDAR (I2C) for forward safety + depth-scale calibration |
| **Proven** | YOLO11m on BPU @ ~10 FPS (`../YOLO11_Webcam.md`), TF-Luna + servo (`../RDK_X5_Peripherals.md`) |

## 📄 Submission documents
- **[PROPOSAL.md](PROPOSAL.md)** — Challenge 1 (Concept), 2 (Architecture), 3 (Engineering Plan), all-in-one
- **[ROADMAP.md](ROADMAP.md)** — week-by-week milestones with exit criteria → Stage 3 demo
- **[docs/bom.md](docs/bom.md)** — full Bill of Materials + power architecture
- **[docs/architecture.md](docs/architecture.md)** — extended diagrams & design decisions
- **[docs/depth_anything_conversion.md](docs/depth_anything_conversion.md)** — Depth Anything → BPU `.bin`
- **[docs/vio_slam_plan.md](docs/vio_slam_plan.md)** — VIO SLAM build & integration plan (W4–W5)
- **[docs/operator_app.md](docs/operator_app.md)** — desktop operator console + robot agent (protocol, safety model)
- **[NEXT_STEPS.md](NEXT_STEPS.md)** — ordered checklist from here to the Stage 3 demo

## Why it's not a stock demo
Most RDK demos use **one** camera, **wheel encoders**, and never lose their pose. This robot has **no
encoders** and explicitly solves the **kidnapped-robot problem**: it fuses **monocular depth (Depth
Anything on the BPU)**, the **3-camera surround**, and an **MPU6050 IMU** into **VIO SLAM**, saves a map,
and **relocalizes** after being lifted and moved. The motor loop is closed via SLAM, not encoders.
See *Innovation* in the proposal.

## Repo layout
```
PROPOSAL.md  ROADMAP.md         # submission entry docs
docs/        # bom, architecture, exported diagram images
src/         # ROS 2 packages (colcon): bringup, msgs, cameras, perception, slam, navigation, drive
app/         # operator app: agent/ (robot WebSocket agent) + desktop/ (PySide6 console)
launch/      # top-level launch files
models/      # compiled BPU .bin models (depth_anything, yolo11)
config/      # YAML params, camera calib, duty<->velocity LUT
scripts/     # bench tests, calibration, bag analysis
```

## 🚀 How to run — obstacle avoidance + ROS navigation

### One-time prerequisites (already applied on the robot)
- `dtoverlay=dtoverlay_pwm3` in `/boot/config.txt` (motor PWM on pins 32/33) + reboot
- `options uvcvideo quirks=128` in `/etc/modprobe.d/uvcvideo-navbot.conf` (3 cameras on one USB-2 bus)
- BPU models: PIDNet-S + Depth Anything builds in `~/Desktop/RDK/model_output_*`,
  YOLO11m in `~/rdk_model_zoo/samples/vision/ultralytics_yolo/model/`
- Cameras by physical port (`/dev/v4l/by-path/...`): **front = port 1.1** (1 MP, MJPEG),
  sides = ports 1.3 / 1.4 (YUYV 320×240 — MJPEG on the side cams saturates the bus)

### Standalone avoidance demos (`scripts/`, no ROS needed)
All serve a live view at `http://<robot-ip>:8080` (camera + perception overlay + decision
banner). Always test **`--dry-run` first** (perception only, motors off); the TF-Luna is a
hard forward stop in every variant. Common flags: `--duty`, `--stop-cm`, `--seconds N`.

| Script | Perception | Loop | Character |
|---|---|---|---|
| `07_pidnet_avoid.py` | PIDNet-S floor segmentation (9 ms/frame) | ~15 Hz | sees floor vs not-floor; lighting-sensitive |
| `08_yolo_avoid.py` | YOLO11m detection (52 ms/frame), widest-gap steering | ~10 Hz | sharp on people/furniture; blind to walls & non-COCO clutter |
| `09_hybrid_avoid.py` | PIDNet floor **and** YOLO object veto, fused per column | ~7.5 Hz | drives only where there is floor AND no object |
| `10_depth_preview.py` | Depth Anything V2 preview (`--model vits392\|vitsopt2`) | 2.9 / 1.2 FPS | side-by-side depth view for model comparison |

```bash
sudo python3 scripts/09_hybrid_avoid.py --dry-run     # watch :8080, then drop --dry-run
```

### ROS 2 navigation stack (PIDNet → planner → safety gate)
```bash
# build (once, or after changes)
source /opt/tros/humble/setup.bash
colcon build --symlink-install

# run — motors OFF by default; verify topics first
source install/setup.bash          # required in EVERY shell (custom navbot_msgs)
ros2 launch navbot_bringup navigation.launch.py

ros2 topic hz /obstacles           # PIDNet fusion, 24 sectors, ~10 Hz
ros2 topic echo /cmd_vel_safe      # what the motors would receive

# robot drives
ros2 launch navbot_bringup navigation.launch.py motors:=true
# kill switch (also: unplug the L298N)
ros2 service call /estop std_srvs/srv/SetBool "{data: true}"
```

Chain: `three_cam` → **`obstacle_fusion`** (PIDNet ×3 cams → `/obstacles`
`navbot_msgs/Sectors`, unknown sector = blocked) → **`local_planner`** (widest free run →
`/cmd_vel`, never forward into blocked) → **`safety_gate`** (TF-Luna < 30 cm or unreadable
blocks forward; `/estop`) → **`motor_controller`** (`motors:=true` only).

Tuning: camera mount angles/FOV in `obstacle_fusion` (`front_axis_deg`, `left_axis_deg`,
`*_hfov_deg`); gap acceptance in `local_planner` (`min_run_deg`, `front_cone_deg`);
stop distance in `safety_gate` (`stop_cm`).

### 🎮 Operator console (desktop app)
Drive and monitor the robot from a Windows/Linux PC — live cameras with the
obstacle-sector HUD, joystick/WASD teleop, mode switching (observe / manual /
auto), E-stop, health panel. Robot side:

```bash
sudo ./app/agent/run_agent.sh        # or: sudo systemctl start navbot-agent
```

PC side: `pip install -r app/desktop/requirements.txt && python -m navbot_console`
(from `app/desktop/`), then connect to `ws://<robot-ip>:8080`.
Details, protocol, and safety model: [docs/operator_app.md](docs/operator_app.md).

## Status
Stage 2 deliverables complete. W1 bring-up **done and verified**: 3 cameras simultaneous
(30/30/16 fps), `/imu/data` @ 100 Hz, motors, TF-Luna. Obstacle avoidance running via three
BPU perception variants (PIDNet / YOLO11 / hybrid) + reactive ROS navigation stack
(`/obstacles` @ 10 Hz, `/cmd_vel_safe` @ 20 Hz, E-stop). Next per roadmap: depth ≥ 5 FPS
(recompile Depth Anything at smaller input), VIO SLAM.

## License
MIT — see [LICENSE](LICENSE).
