# Roadmap — RDK X5 Tri-Cam NavBot

> **Version:** 1.0 &nbsp;|&nbsp; **Date:** 2026-06-26 &nbsp;|&nbsp; **Target:** Stage 3 demo (end of Week 7)

Week-by-week milestones with **exit criteria** (the test that must pass before moving on). This doubles
as the GitHub Projects board: each row = a milestone column, each exit criterion = a closing issue.

| Week | Milestone | Key tasks | Exit criteria (must pass) |
|---|---|---|---|
| **W1** | **3-camera + IMU bring-up** | `hobot_usb_cam` ×3 via the USB 3.0 hub (`navbot_cameras` ✓); MPU6050 driver on I2C5 → `/imu/data`; repo skeleton ✓ | 3 `*/image_raw` publish **simultaneously** (front ≥ 25 fps, sides ≥ 12 fps); `/imu/data` ≥ 100 Hz; `ros2 topic hz` confirms |
| **W2** | **Drive + safety** | Wire L298N; `motor_controller` ✓ `Twist`→duty; `safety_gate` + E-stop; **duty↔velocity LUT** per surface | Robot drives fwd/back/turn from `/cmd_vel`; E-stop zeros motors < 100 ms; LUT saved to `config/drive_lut.yaml` ✓ |
| **W3** | **Depth Anything on BPU** | Convert Depth Anything ONNX→`.bin` (`docs/depth_anything_conversion.md`); `depth_bpu` ROS node; re-add YOLO11 as `detection_bpu` time-shared | `/perception/depth` ≥ 5 FPS on BPU; depth vs TF-Luna error ≤ 15 % at 1–3 m; detection still ≥ 3 FPS |
| **W4** | **VIO SLAM + mapping** | `vio_slam` (front cam + IMU + depth) → `/odom`+`/tf`+`/map`; `/save_map`,`/load_map`; obstacle_fusion v1 | Map a ≥ 30 m² room with ≥ 1 loop closure; drift ≤ 5 % on a 10 m loop; map reloads from disk |
| **W5** | **Relocalization + navigation** | `relocalizer` (kidnap recovery vs saved map); `behaviour_manager` (MAP/NAV/AVOID/RELOCALIZE); `local_planner` | After lift-and-move, relocalize ≤ 10 s & ≤ 30 cm/15°; reach saved goal from 3 m avoiding 1 obstacle ≥ 6/10 |
| **W6** | **Integration & real-time** | Core pinning + RT prio on safety/motor; BPU depth/detection scheduling; drop-not-queue; latency profiling | p95 latency image→`/cmd_vel` ≤ 150 ms; 0 hard collisions in 5 min soak; 20 min battery run |
| **W7** | **Stage 3 demo** | Scored mission: map → navigate to saved goal **incl. a kidnap** → arrive; record bag + video; finalize docs/dashboard | **≥ 8/10** mission success (with kidnap); bag + annotated video committed; docs tagged `v1.0-demo` |

## Dependencies / critical path
```
W1 cam+IMU ──► W3 depth(BPU) ──► W4 VIO SLAM ──► W5 reloc+nav ──► W6 integration ──► W7 demo
                                      ▲
W2 drive+safety ──────────────────────┘   (drive can proceed in parallel after W1)
```
W2 (drive) runs in parallel once W1 lands. SLAM (W4) depends on both depth (W3) and the IMU (W1).
W6 is the schedule buffer — if any earlier week slips, W6 absorbs it before the demo.

## Stretch goals (only if W1–W6 land early)
- Semantic mapping: tag detections (YOLO11) onto the SLAM map for "go to the chair" goals.
- Multi-camera relocalization using the side cameras for wider place recognition.
- Web dashboard with live `/map` + `/odom` trail + depth + detection overlay.

## GitHub Projects mapping
- One **milestone** per week (W1…W7).
- Each exit criterion → one **issue** labelled `exit-criteria`, closed only when the test passes.
- Columns: `Backlog → In progress → Blocked → Review → Done`.
- Public board link goes in the repo `README.md` and the submission PR.
