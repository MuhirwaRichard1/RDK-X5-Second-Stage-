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
launch/      # top-level launch files
models/      # compiled BPU .bin models (depth_anything, yolo11)
config/      # YAML params, camera calib, duty<->velocity LUT
scripts/     # bench tests, calibration, bag analysis
```

## Status
Stage 2 deliverables complete (concept + architecture + plan). Implementation follows the roadmap.

## License
MIT — see [LICENSE](LICENSE).
