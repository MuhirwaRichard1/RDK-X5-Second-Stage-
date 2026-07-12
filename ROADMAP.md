# Roadmap — RDK X5 Tri-Cam NavBot

> **Version:** 1.0 &nbsp;|&nbsp; **Date:** 2026-06-26 &nbsp;|&nbsp; **Target:** Stage 3 demo (by 10th July)
>
> **2026-07-13 — plan superseded:** the sensor architecture pivoted from visual SLAM
> (TF-Luna + cameras, then briefly RealSense D430) to an **RPLidar C1** driving both
> SLAM (slam_toolbox) and obstacle avoidance. This table is kept as the original
> submission; the live, updated plan is **[NEXT_STEPS.md](NEXT_STEPS.md)**.

Week-by-week milestones with **exit criteria** (the test that must pass before moving on). This doubles
as the GitHub Projects board: each row = a milestone column, each exit criterion = a closing issue.

| Week | Milestone | Key tasks | Exit criteria (must pass) |
|---|---|---|---|
| **2 days** | **3-camera + IMU bring-up** | `hobot_usb_cam` ×3 via the USB 3.0 hub (`navbot_cameras` ✓); MPU6050 driver on I2C5 → `/imu/data`; repo skeleton ✓ | 3 `*/image_raw` publish **simultaneously** (front ≥ 25 fps, sides ≥ 12 fps); `/imu/data` ≥ 100 Hz; `ros2 topic hz` confirms |
| **2 days** | **Drive + safety** | Wire L298N; `motor_controller` ✓ `Twist`→duty; `safety_gate` + E-stop; **duty↔velocity LUT** per surface | Robot drives fwd/back/turn from `/cmd_vel`; E-stop zeros motors < 100 ms; LUT saved to `config/drive_lut.yaml` ✓ |
| **2 days** | **Depth Anything on BPU** | Convert Depth Anything ONNX→`.bin` (`docs/depth_anything_conversion.md`); `depth_bpu` ROS node; re-add YOLO11 as `detection_bpu` time-shared | `/perception/depth` ≥ 5 FPS on BPU; depth vs TF-Luna error ≤ 15 % at 1–3 m; detection still ≥ 3 FPS |
| **2 days** | **VIO SLAM + mapping** | `vio_slam` (front cam + IMU + depth) → `/odom`+`/tf`+`/map`; `/save_map`,`/load_map`; obstacle_fusion v1 | Map a ≥ 30 m² room with ≥ 1 loop closure; drift ≤ 5 % on a 10 m loop; map reloads from disk |
| **2 days** | **Relocalization + navigation** | `relocalizer` (kidnap recovery vs saved map); `behaviour_manager` (MAP/NAV/AVOID/RELOCALIZE); `local_planner` | After lift-and-move, relocalize ≤ 10 s & ≤ 30 cm/15°; reach saved goal from 3 m avoiding 1 obstacle ≥ 6/10 |
| **2 days** | **Integration & real-time** | Core pinning + RT prio on safety/motor; BPU depth/detection scheduling; drop-not-queue; latency profiling | p95 latency image→`/cmd_vel` ≤ 150 ms; 0 hard collisions in 5 min soak; 20 min battery run |
| **2 days** | **Stage 3 demo** | Scored mission: map → navigate to saved goal **incl. a kidnap** → arrive; record bag + video; finalize docs/dashboard | **≥ 8/10** mission success (with kidnap); bag + annotated video committed; docs tagged `v1.0-demo` |

## Dependencies / critical path
```
W1 cam+IMU ──► W1 depth(BPU) ──► W1 VIO SLAM ──► W1.5 reloc+nav ──► W2 integration ──► W2 demo
                                      ▲
W2 drive+safety ──────────────────────┘   (drive can proceed in parallel after W
```
W1 (drive) runs in parallel once W1 lands. SLAM (W1) depends on both depth (W1.5) and the IMU (W1).
W2 is the schedule buffer — if any earlier week slips, W6 absorbs it before the demo.

## Stretch goals (only if W1–W2 land early)
- Semantic mapping: tag detections (YOLO11) onto the SLAM map for "go to the chair" goals.
- Multi-camera relocalization using the side cameras for wider place recognition.
- Web dashboard with live `/map` + `/odom` trail + depth + detection overlay.

## GitHub Projects mapping
- One **milestone** per week (W1…W2).
- Each exit criterion → one **issue** labelled `exit-criteria`, closed only when the test passes.
- Columns: `Backlog → In progress → Blocked → Review → Done`.
- Public board link goes in the repo `README.md` and the submission PR.
