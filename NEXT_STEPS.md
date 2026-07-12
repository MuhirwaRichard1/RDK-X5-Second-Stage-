# NEXT STEPS — your step-by-step path to the Stage 3 demo

> Updated 2026-07-13, after the **lidar pivot**: TF-Luna + visual SLAM (Track A) and the
> RealSense D430 detour are both retired — an **RPLidar C1** now drives SLAM *and*
> obstacle avoidance. This file is the live plan; ROADMAP.md / PROPOSAL.md are the
> original (pre-pivot) submissions and stay as history.
>
> **Already done & verified** (skip, don't redo):
> - **Platform:** 3 cameras simultaneous (front MJPEG + YUYV sides), `/imu/data` ~190 Hz,
>   motors + duty↔velocity LUT (`invert_linear` fixes the swapped harness), camera
>   intrinsics calibrated.
> - **Operator app:** desktop console (PySide6, installers via CI) + robot agent
>   (systemd, WS+UDP :8080, modes stopped/observe/manual/auto, teleop, E-stop,
>   model toggles, camera tiles, telemetry HUD).
> - **RPLidar C1** on `/dev/ttyUSB0` @ 460800: sllidar_ros2 vendored in `src/`
>   (gitignored — reclone on fresh checkout), `/scan` 10 Hz / 720 pts / 16 m.
>   **Mount:** laser 0° faces the robot's REAR → `yaw_offset_deg: 180` (scan_sectors,
>   safety_gate) and `lidar_yaw: π` (lidar_slam TF) must stay in sync.
> - **Lidar SLAM online:** `lidar_slam.launch.py` = dr_odom (dead-reckoning backbone:
>   /cmd_vel_safe + IMU gyro, square closes 0.7 cm/0.9°) + slam_toolbox async +
>   base_link→laser TF. Works live and on bag replay (`use_sim_time:=true`).
> - **Obstacle avoidance (lidar, zero BPU):** `scan_sectors` → `/obstacles` (26×10°
>   over ±130°, BLOCKED < 0.5 m); `safety_gate` = always-on 30 cm forward stop (±15°)
>   + always-on 60 cm-⌀ proximity ring (blocks motion *toward* intruders; rotation
>   always passes) + operator-togglable sector stop + **manual-mode steering assist**
>   (deviates toward the clearer side < 0.8 m ahead). Verified with a 7-scenario
>   fake-scan behavioral suite. The old PIDNet / Depth-Anything obstacle pipelines
>   are deleted.
> - **Console MAP view:** MAP toggle → agent renders `/map` to PNG @ 1 Hz with the
>   robot marker from the map→base_link TF (needs SLAM running).
> - **Depth Anything demo overlay:** `depth_bpu` fixed to the **vits392** bin (the
>   only variant that fits the 320 MB ION pool); DEPTHANYTHING toggle works, 2.5 Hz.
> - **First real map:** `maps/arena_20260713` (.pgm/.yaml + .posegraph/.data) from a
>   5 min online drive; raw bag at `bags/mapping_fresh_20260713` for offline rebuilds.

---

## Step 0 — Commit & push the working tree (10 min) ⚠️ do this first
Uncommitted right now: the `depth_bpu` vits392 fix and `maps/arena_20260713.*`.
```bash
cd ~/rdk-x5-navbot
git add -A && git status      # review; the 7 MB .posegraph is optional to track
git commit -m "depth_bpu vits392 fix + first arena map" && git push
```

## Step 1 — Demo-quality map of the full arena (1–2 h)
The current map covers ~8.5 m² — remap the whole demo area in one continuous drive.
1. Manual mode on, then fresh SLAM + recording (ask Claude, or):
   ```bash
   ros2 launch navbot_slam lidar_slam.launch.py   # agent mode already owns the lidar
   ros2 bag record /scan /imu/data /cmd_vel /cmd_vel_safe -o bags/arena_full
   ```
2. Drive: slider ≥ 30 %, gentle turns, pause at corners, **close every loop** (return
   to the start; revisit rooms from the same direction).
3. Save + snapshot:
   ```bash
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
     "{name: {data: /home/sunrise/rdk-x5-navbot/maps/arena_demo}}"
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: /home/sunrise/rdk-x5-navbot/maps/arena_demo}"
   ```
4. **Pass:** walls straight and single (no doubling), every demo room covered, ≥ 1 loop
   closure observed live (map visibly snaps). If it smears: rebuild offline from the
   bag with tuned slam_toolbox params instead of re-driving.

## Step 2 — Localization mode against the saved map (half day)
Run slam_toolbox in `localization` mode with `map_file_name: maps/arena_demo` (new
`localization:=true` arg in `lidar_slam.launch.py`, or a second config yaml).
**Pass:** started anywhere in the arena, the console marker snaps to the true position
after a short drive (≤ 10 s), and stays correct through a 2-min teleop.

## Step 3 — Goal navigation (2 days)
`behaviour_manager` + goal-seeking planner: send `/goal` (console click on the MAP
panel is the natural UI — the agent already has the map→pixel transform), plan through
map free space, follow with `local_planner`-style sector steering; safety_gate stays
underneath untouched.
**Pass:** reach a saved goal from 3 m away around 1 obstacle, ≥ 6/10 runs.

## Step 4 — Kidnap recovery (1–2 days)
Lift detect (IMU |az| spike + /scan discontinuity) → E-stop → on set-down, rotate-scan
until localization converges → resume goal.
**Pass:** after lift-and-carry-3 m, relocalize ≤ 10 s within 30 cm / 15°.

## Step 5 — Integration & real-time hardening (2 days)
- E-stop zeroes motors < 100 ms under full load (measure over UDP path).
- p95 `/scan`→`/cmd_vel_safe` latency ≤ 150 ms; queue depth 1 everywhere on images.
- 5-min soak with 0 hard collisions; 20-min battery run (watch `dmesg` for USB resets
  — hub power under motor load; the right-cam connector is already known-loose).

## Step 6 — Stage 3 demo (1–2 days)
Scored mission, ≥ 8/10: map (or load `arena_demo`) → navigate to saved goal →
kidnap mid-run → relocalize → arrive. Record rosbag + annotated video of every run,
commit both, tag `v1.0-demo`, update README Status.

---

### Quick reference — daily drivers
```bash
# modes are owned by the agent (console buttons, or WS :8080 set_mode)
ros2 launch navbot_slam lidar_slam.launch.py            # SLAM beside an active mode
ros2 service call /estop std_srvs/srv/SetBool "{data: true}"      # kill switch
ros2 topic hz /scan /obstacles /odom_dr                 # sensor health
ros2 bag record /scan /imu/data /cmd_vel /cmd_vel_safe -o bags/<name>
```

### Known traps (cost hours before — don't rediscover them)
- Source **both** `/opt/tros/humble/setup.bash` **and** `install/setup.bash`.
- The agent owns ONE `ros2 launch` — **never** start manual/navigation launches by
  hand next to a live mode (camera abort -6, GPIO unexport kills the live
  motor_controller, and now the lidar serial port collides too). Apply rebuilds via
  `set_mode stopped → manual`.
- Console **speed slider scales linear velocity only** — mapping at < 20 % slider
  gives creep-forward with normal-speed turns and ruins scan matching.
- C1 mount: laser 0° = robot rear. Change the mount → update `yaw_offset_deg` (×2
  launches) **and** `lidar_yaw` together.
- Keep the robot **still ~3 s** after starting imu_driver / dr_odom (gyro bias).
- Side cameras must stay **YUYV**; front camera lives in port 1.1 (by-path names).
- BPU/ION: only single-subgraph bins load; vits392 is the proven depth model; YOLO11
  and Depth Anything may not fit ION together — toggle one off first.
- MAP view is blank until slam_toolbox runs; the PNG only refreshes when the grid
  changes (drive a little).
- Root-run tools leave root-owned files in sunrise's repo — `chown` back.
- `pkill -f <pattern>` self-matches the compound command that contains the pattern —
  use `[c]haracter-class` patterns.
