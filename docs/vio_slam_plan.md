# VIO SLAM — Build & Integration Plan

> **Version:** 1.0 | **Date:** 2026-07-06 | Target: ROADMAP **W4** (map + odom + loop closure)
> and **W5** (kidnap relocalization). Exit criteria: map a ≥30 m² room with ≥1 loop closure,
> drift ≤5 % on a 10 m loop, map reloads from disk, robot relocalizes after being lifted.

## 0. What we are designing around (measured facts, 2026-07-06)

| Reality | Consequence for SLAM |
|---|---|
| No wheel encoders | Odometry must come entirely from vision + IMU |
| Front cam = $10-class 1 MP UVC, **rolling shutter**, **no hardware sync**, MJPEG 30 fps | Tightly-coupled VIO (VINS/ORB3-inertial) will fight shutter distortion + timestamp jitter; cap rotation speed while tracking |
| Camera intrinsics **never calibrated** | Phase 0 blocker — nothing downstream works without this |
| IMU = **clone MPU6050**: accel scale ×1.79 (auto-corrected in `imu_driver`), gyro OK, 100 Hz Python-polled I2C | Gyro is trustworthy, accel only after correction; timestamps jitter ~ms; can raise rate to 200 Hz |
| Depth Anything vits392 = **2.9 FPS**, relative (not metric) depth | Depth-assisted SLAM needs metric scaling (TF-Luna) and can't key every frame off depth |
| TF-Luna forward range, verified | Free metric anchor: scale depth maps + sanity-check map scale |
| 8× A55 CPU (~3.6 GB free), BPU busy-capable | SLAM budget realistically 2–3 cores; keep 640×480 processing |
| `ros-humble-rtabmap-ros` 0.23.7 + `robot_localization` **installable via apt** | A loosely-coupled RGB-D pipeline is one `apt install` away; ORB-SLAM3 is a half-day source build |

**Strategy: two tracks, one decision gate.** Track A (RTAB-Map, loosely-coupled RGB-D + IMU)
is the primary because it tolerates our sync/shutter weaknesses, and its loop closure,
map save/load, and global relocalization directly cover the W4+W5 exit criteria. Track B
(ORB-SLAM3 mono-inertial) is the fallback/upgrade if Track A odometry proves too weak.
Both are evaluated **offline on the same recorded bags** before anything runs live.

## Phase 1 — Sensor foundation (≈2 days) — *prerequisite for everything*

1. **Camera intrinsics** — tooling DONE (`scripts/11_front_cam_calib.py`: live chessboard
   capture on :8080, auto-writes `config/camera_front.yaml`; `three_cam.launch.py` loads it
   automatically once the file exists). **TODO (human): print a 9×6-inner-corner board,
   run the capture, RMS < 0.5 px.** Repeat at 1280×720 if SLAM uses 720p.
2. **IMU hardening** — ✅ DONE 2026-07-06: driver at **200 Hz** (verified `ros2 topic hz`
   = 200.0), timestamp taken at I2C read, covariances = measured values.
   Noise measured with `scripts/12_imu_noise.py` (607 Hz raw sampling, 0.22 ms jitter,
   120 s): gyro 7.2e-5 rad/s/√Hz, accel 3.3e-3 m/s²/√Hz — **datasheet-grade**; the clone
   chip's only defect is the accel scale (×1.795, auto-corrected). SLAM noise config can
   use these numbers directly.
3. **Camera–IMU extrinsics**: mount both rigidly (any flex kills VIO); measure the lever arm
   with calipers + note axis orientations. Refine later with Kalibr on a recorded bag only
   if Track B is pursued (Track A mostly needs orientation alignment).
4. **Time offset** — tooling DONE (`scripts/13_cam_imu_offset.py`: optical-flow ↔ gyro-z
   cross-correlation). **TODO (human): run 3× while rotating the robot by hand; use the
   mean; expect |offset| < 50 ms.**

## Phase 2 — Data pipeline + recorded test set (≈1 day)

1. Decode path — **decision made, verified 2026-07-06: use the X5's hardware JPEG codec.**
   `hobot_codec` decodes the front cam's 720p MJPEG → NV12 at 30 Hz using ~11 % of one
   A55 core (vs most of a core for `cv2.imdecode`):
   ```
   ros2 launch hobot_codec hobot_codec_decode.launch.py \
     codec_in_mode:=ros codec_in_format:=jpeg \
     codec_out_mode:=ros codec_out_format:=nv12 \
     codec_sub_topic:=/cam_front/image_raw codec_pub_topic:=/cam_front/nv12
   ```
   SLAM consumes the Y plane of NV12 directly as mono8 (zero conversion cost); the only
   CPU step left is rectification (`cv2.remap` with precomputed maps, ~1–2 ms at 640×480).
   Transport: `hobot_shm` (FastDDS shared memory) is already enabled by
   `three_cam.launch.py` and benefits ALL intra-host nodes including Python ones; full
   zero-copy (`codec_out_mode:=shared_mem` → `/hbmem_img`) is available end-to-end
   between C++ TROS nodes — use it if the SLAM front-end is C++ (Track B / RTAB-Map),
   fall back to `ros` out-mode for Python consumers (rclpy cannot subscribe hbmem).
2. Metric depth node (`navbot_perception/depth_bpu`, W3 item): vits392 → relative depth →
   scale via TF-Luna (center-pixel regression, per ROADMAP: error ≤15 % at 1–3 m) →
   `/perception/depth` (32FC1, ~3 Hz — after the planned smaller-input recompile, ≥5 Hz).
3. **Record canonical bags** (rosbag2: front cam, camera_info, /imu/data, depth, TF-Luna):
   - `loop_10m`: tape-measured 10 m loop, gentle motion — drift metric,
   - `room_30m2`: full room sweep with revisits — loop-closure metric,
   - `kidnap`: drive, lift, carry 3 m, set down — relocalization metric,
   - `stress`: fast rotation, low light — failure characterization.
   These bags are the regression suite; every tuning change replays against them.

## Phase 3 — Backend bake-off, offline (≈3 days)

- **Track A**: `apt install ros-humble-rtabmap-ros`. `rgbd_odometry` (frame-to-map, feature
  odometry on 30 fps RGB with 3 Hz registered depth) + `imu_filter_madgwick` orientation
  prior + `rtabmap` back-end (loop closure, `map → odom` TF, database on disk).
  Key risk: odometry between depth updates — mitigate with gyro prior + planner speed caps.
- **Track B**: build ORB-SLAM3 (+Pangolin headless) on-device (~half day), ROS 2 wrapper,
  mono-inertial on the same bags. Expect time-offset and rolling-shutter sensitivity;
  online `td` estimation on.
- **Score both** on the bags: 10 m-loop drift %, tracking-loss count, loop-closure success,
  relocalization success on `kidnap`, CPU %. **Gate: pick the track that meets
  drift ≤5 % with fewest losses; ties → Track A (simpler ops, native save/load).**

## Phase 4 — Online integration as `navbot_slam/vio_slam` (≈3 days)

Wrap the winner behind the interfaces the rest of the robot already expects
(`navbot_slam` README): `/odom` (nav_msgs/Odometry, ≥10 Hz) + TF `map → odom → base_link`,
`/map` (occupancy grid from RTAB-Map, or keyframe cloud projection for ORB3),
`/save_map` + `/load_map` (navbot_msgs MapIO), tracking-lost → `RELOCALIZE` status topic.
Optionally fuse `/odom` + `/imu/data` in `robot_localization` EKF for a smoother 30 Hz
odom for the planner. Add `slam.launch.py` to `navbot_bringup` layering on
`navigation.launch.py`; planner gains speed caps while tracking (rolling shutter).

## Phase 5 — Kidnap recovery (`relocalizer`, ≈2 days)

Lift detection = accel magnitude spike + PIDNet floor disappearing. On lift: E-stop via
`/estop`, mark pose invalid. On set-down (accel settles): rotate-in-place scan while the
backend attempts global relocalization against the loaded map (RTAB-Map does this natively);
publish `/relocalized_pose`, release E-stop, resume behaviour. Validate on the `kidnap`
bag first, then live.

## Phase 6 — Validation against ROADMAP (≈1 day)
Tape-measured 10 m loop (drift ≤5 %), 30 m² room map with ≥1 loop closure confirmed in the
RTAB-Map DB, reboot + `/load_map` + relocalize, 5× kidnap trials. Record demo footage.

## Risk register

| Risk | Likelihood | Mitigation |
|---|---|---|
| Rolling shutter breaks tracking on fast turns | High | Planner caps `w` while SLAM active; prefer Track A (per-frame features, no tight IMU coupling) |
| Camera/IMU time jitter (USB + Python) | High | Timestamp at read; constant-offset calibration; loosely-coupled first |
| Clone-IMU accel bias drift | Medium | Gyro-dominant fusion; accel only for gravity/orientation prior |
| Depth too slow (2.9 FPS) for RGB-D odometry | Medium | W3 recompile at ~336 input → ≥5 FPS; meanwhile odometry keys on RGB, depth only registers keyframes |
| CPU saturation (SLAM + PIDNet + YOLO) | Medium | Drop YOLO during mapping runs; 640×480; monitor with `top` in every bag replay |
| ORB-SLAM3 build pain on this image | Low | Only pursued if Track A fails the gate |

**Total: ~2 weeks** matching ROADMAP W4–W5, with the bake-off gate at the end of week 1.
