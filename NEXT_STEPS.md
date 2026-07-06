# NEXT STEPS — your step-by-step path to the Stage 3 demo

> Updated 2026-07-06. Ordered checklist of what **you** still have to do, with commands
> and pass criteria. Detailed rationale: [docs/vio_slam_plan.md](docs/vio_slam_plan.md).
>
> **Already done & verified** (skip, don't redo): 3 cameras simultaneous (30/30/16 fps),
> `/imu/data` @ 200 Hz with measured noise covariances, motors + LUT, TF-Luna + MPU6050 on
> I2C5, obstacle avoidance (PIDNet / YOLO / hybrid, scripts 07–09), ROS navigation chain
> (`/obstacles` 10 Hz → planner → safety_gate + E-stop), HW JPEG decode path, depth model
> bake-off (use **vits392**; the 25/13-subgraph builds can't load — don't fight them).

---

## Step 0 — Commit your work (10 min) ⚠️ do this first
Everything since `764f314` is uncommitted: navigation stack, avoidance scripts 07–10,
calibration tools 11–13, safety_gate, docs. Commit now so every later step has a
known-good baseline to diff against:
```bash
cd ~/rdk-x5-navbot
git add -A && git status        # review — no *.bin/*.log/calib_frames junk
git commit -m "obstacle avoidance (PIDNet/YOLO/hybrid), ROS nav stack, SLAM phase 1 tooling"
git push
```

## Step 1 — Camera intrinsics calibration (30 min, needs a printed chessboard)
1. Print a chessboard with **9×6 inner corners**, tape it dead-flat to cardboard,
   measure one square with a ruler (mm).
2. ```bash
   python3 scripts/11_front_cam_calib.py --square-mm <measured>
   ```
3. Open `http://<robot-ip>:8080`, move the board: near/far, all corners, strong tilts,
   until 40 auto-captures (green flash each).
4. **Pass:** printed `RMS reprojection error < 0.5 px`. Re-run with better coverage if not.
   Output `config/camera_front.yaml` is auto-loaded by the camera launch from then on.

## Step 2 — Camera–IMU time offset (15 min)
```bash
sudo python3 scripts/13_cam_imu_offset.py     # rotate robot left-right by hand, ~1 Hz
```
Run **3×**, note the mean. **Pass:** |offset| < 50 ms, repeatable ±5 ms.
Write the value at the top of `docs/vio_slam_plan.md` — the SLAM config needs it later.

## Step 3 — Rigid mount + lever arm (30 min, mechanical)
Bolt (don't tape) the front camera and IMU to the chassis — any flex ruins VIO.
Measure with calipers: IMU→camera x/y/z offsets (mm) and note both axis orientations.
Record them in `docs/vio_slam_plan.md` §Phase 1.3.

## Step 4 — Depth Anything ≥ 5 FPS recompile (half day, PC with the toolchain)
vits392 (2.9 FPS) misses the ≥5 FPS exit criterion. On the machine you used for the
existing conversions (see `docs/depth_anything_conversion.md`):
1. Re-export Depth Anything V2 ViT-S at input **~336×336** (or 280), same recipe that
   produced your **single-subgraph** builds (vits392/vitsopt2) — *not* the "opt" recipe
   that fragmented into 13–25 subgraphs.
2. Copy `model_output_*` to `~/Desktop/RDK/`, then verify on the robot:
   ```bash
   ls ~/Desktop/RDK/model_output_<new>/main_graph_subgraph_*.json | wc -l   # must be 1
   sudo python3 scripts/10_depth_preview.py --model ~/Desktop/RDK/model_output_<new>/<file>.bin
   ```
3. **Pass:** ≥ 5 FPS in the preview banner and depth image looks sane at :8080.

## Step 5 — depth_bpu node + canonical bags (1 day)
1. Ask Claude to build `navbot_perception/depth_bpu` (new model → metric scale via
   TF-Luna → `/perception/depth`). **Pass:** `ros2 topic hz /perception/depth` ≥ 5,
   depth vs TF-Luna error ≤ 15 % at 1–3 m (point robot at a wall, compare).
2. Record the four regression bags (cameras+IMU+depth running; drive with keyboard or
   slow `motors:=true`):
   - `bags/loop_10m` — tape-measure a 10 m loop on the floor, drive it gently
   - `bags/room_30m2` — full room sweep, end where you started
   - `bags/kidnap` — drive, lift, carry 3 m, set down, drive again
   - `bags/stress` — fast turns, lights dimmed
   ```bash
   ros2 bag record /cam_front/image_raw /cam_front/camera_info /imu/data /perception/depth -o bags/loop_10m
   ```
   These bags are your offline test set for every SLAM decision that follows.

## Step 6 — SLAM bake-off on the bags (2–3 days)
1. `sudo apt install ros-humble-rtabmap-ros ros-humble-imu-filter-madgwick`
2. With Claude: wire RTAB-Map (rgbd_odometry + imu prior) to replay each bag; measure
   10 m-loop drift %, tracking losses, loop closures, kidnap relocalization, CPU %.
3. Only if drift > 5 % or tracking is unusable: build ORB-SLAM3 (Track B) and compare.
4. **Gate: pick the backend that meets drift ≤ 5 % with fewest losses.** Record numbers
   in the plan doc.

## Step 7 — Online `vio_slam` integration (2 days)
With Claude: wrap the winner as `navbot_slam/vio_slam` publishing `/odom` + TF +
`/map`, `/save_map` + `/load_map`; add `slam.launch.py`; planner speed caps while
tracking. **Pass (= W4 exit):** map a ≥ 30 m² room live with ≥ 1 loop closure; drift
≤ 5 % on the 10 m loop; map survives reboot + `/load_map`.

## Step 8 — Kidnap recovery + goal navigation (2 days)
1. `relocalizer`: lift detect (IMU spike + floor vanishing) → E-stop → on set-down,
   rotate-scan until the backend relocalizes → `/relocalized_pose` → resume.
2. `behaviour_manager`: MAP / NAV / AVOID / RELOCALIZE states, `/goal` handling; planner
   seeks the goal through free sectors instead of wandering.
3. **Pass (= W5 exit):** after lift-and-carry-3 m, relocalize ≤ 10 s within 30 cm/15°;
   reach a saved goal from 3 m away around 1 obstacle, ≥ 6/10 runs.

## Step 9 — Integration & real-time hardening (2 days)
- Pin safety_gate + motor_controller to dedicated cores, RT priority; verify E-stop
  zeroes motors < 100 ms under full load.
- Confirm queue depth 1 (drop-not-queue) on every image subscriber.
- Profile p95 image→`/cmd_vel` latency ≤ 150 ms (`ros2 topic delay` + timestamps).
- 5-min soak with 0 hard collisions; 20-min battery run without brownout
  (watch `dmesg` for USB resets — the hub power matters under motor load).

## Step 10 — Stage 3 demo (1–2 days)
Scored mission, ≥ 8/10: map the room → save goal → kidnap mid-run → relocalize →
arrive at goal. Record rosbag + annotated video of every run, commit both, tag
`v1.0-demo`, update README Status.

---

### Quick reference — daily drivers
```bash
ros2 launch navbot_bringup navigation.launch.py [motors:=true]   # avoidance stack
ros2 service call /estop std_srvs/srv/SetBool "{data: true}"     # kill switch
sudo python3 scripts/09_hybrid_avoid.py --dry-run                # standalone avoider + :8080 view
sudo python3 scripts/10_depth_preview.py --model vits392         # depth check
sudo python3 scripts/test_i2c_sensors.py --bus 5                 # IMU+lidar bench test
```
### Known traps (cost hours before — don't rediscover them)
- Source **both** `/opt/tros/humble/setup.bash` **and** `install/setup.bash` in every shell.
- Side cameras must stay **YUYV** — any MJPEG on the 8022 clones starves the USB bus.
- Front camera lives in **port 1.1**; device paths are by-path, not /dev/videoN.
- Keep the robot **still for ~2 s** after starting imu_driver (bias + scale calibration).
- Multi-subgraph BPU builds (13–25 subgraphs) will not load — 382 MB ION is fixed.
