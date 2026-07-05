#!/usr/bin/env python3
"""Step 13 - camera-IMU time offset estimation (VIO SLAM plan, Phase 1.4).

While this runs, pick the robot up and rotate it LEFT-RIGHT (yaw) smoothly,
back and forth, ~1 Hz, for the whole capture. The script records front-camera
frames + gyro-z directly (no ROS), computes per-frame horizontal optical
flow, and cross-correlates it against the gyro to find the constant time
offset between camera and IMU timestamps.

Convention: positive offset = camera timestamps LAG the IMU (image content
is older than its stamp says). Feed the result to the SLAM backend
(RTAB-Map `wait_imu_to_init`/topic remap tolerates small offsets; ORB-SLAM3
/ VINS use it as `td`).

Run:      sudo python3 scripts/13_cam_imu_offset.py --seconds 15
Pass if:  |offset| < 50 ms and repeatable within ~5 ms across 3 runs.
"""
import argparse
import importlib.util
import os
import struct
import threading
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np
from smbus2 import SMBus

_spec = importlib.util.spec_from_file_location(
    "pidnet_avoid", os.path.join(os.path.dirname(__file__), "07_pidnet_avoid.py"))
_pid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pid)
CamGrabber, BY_PATH = _pid.CamGrabber, _pid.BY_PATH

ADDR = 0x68


def gyro_thread(bus, out, stop):
    bus.write_byte_data(ADDR, 0x6B, 0x01)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, 0x1A, 0x02)
    bus.write_byte_data(ADDR, 0x1B, 0x00)
    while not stop.is_set():
        raw = bus.read_i2c_block_data(ADDR, 0x43, 6)   # gyro only
        t = time.monotonic()
        _gx, _gy, gz = struct.unpack(">3h", bytes(raw))
        out.append((t, gz / 131.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--max-lag-ms", type=float, default=200.0)
    args = ap.parse_args()

    cam = CamGrabber("front", BY_PATH.format("1.1"), "MJPG", (640, 480))
    cam.start()
    gyro, stop = [], threading.Event()
    gt = threading.Thread(target=gyro_thread, args=(SMBus(5), gyro, stop),
                          daemon=True)
    gt.start()
    time.sleep(2.0)

    print(f"capturing {args.seconds:.0f}s — rotate the robot LEFT-RIGHT (yaw), "
          f"smooth ~1 Hz sweeps, NOW")
    frames, ftimes = [], []
    last_stamp = 0.0
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        f = cam.latest(max_age=0.2)
        if f is None or cam.stamp == last_stamp:
            time.sleep(0.005)
            continue
        last_stamp = cam.stamp
        small = cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (160, 120))
        frames.append(small)
        ftimes.append(cam.stamp)                      # grab-time stamp
    stop.set()
    gt.join(timeout=1.0)
    cam.stop_evt.set()
    cam.join(timeout=2.0)
    print(f"{len(frames)} frames, {len(gyro)} gyro samples")
    if len(frames) < 60:
        print("too few frames — is the camera up?")
        return 1

    # per-frame-pair mean horizontal flow (px/s), timestamped at pair midpoint
    flow_v, flow_t = [], []
    for i in range(1, len(frames)):
        dt = ftimes[i] - ftimes[i - 1]
        if not 0.01 < dt < 0.2:
            continue
        fl = cv2.calcOpticalFlowFarneback(frames[i - 1], frames[i], None,
                                          0.5, 2, 15, 2, 5, 1.1, 0)
        flow_v.append(float(fl[..., 0].mean()) / dt)
        flow_t.append((ftimes[i] + ftimes[i - 1]) / 2.0)
    flow_v, flow_t = np.asarray(flow_v), np.asarray(flow_t)
    gyro = np.asarray(gyro)

    if np.abs(gyro[:, 1]).max() < 20:
        print("gyro-z peak < 20 deg/s — you did not rotate enough; re-run")
        return 1

    # cross-correlate flow(t) with gyro(t + lag) over candidate lags
    lags = np.arange(-args.max_lag_ms, args.max_lag_ms + 1, 2) / 1000.0
    scores = []
    fv = (flow_v - flow_v.mean()) / (flow_v.std() + 1e-9)
    for lag in lags:
        gz = np.interp(flow_t + lag, gyro[:, 0], gyro[:, 1])
        gz = (gz - gz.mean()) / (gz.std() + 1e-9)
        scores.append(float(np.dot(fv, gz)) / len(fv))
    scores = np.asarray(scores)
    best = int(np.abs(scores).argmax())
    print(f"\ncamera-IMU time offset: {lags[best]*1000:+.0f} ms "
          f"(correlation {scores[best]:+.2f})")
    print("positive = camera stamps lag the IMU. Re-run 2-3x; use the mean.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
