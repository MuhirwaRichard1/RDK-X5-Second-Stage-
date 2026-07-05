#!/usr/bin/env python3
"""Step 8 - tri-camera obstacle avoidance with YOLO11 detection on the BPU.

YOLO-only variant of 07_pidnet_avoid.py. Instead of drivable-surface
segmentation, obstacles are COCO objects detected by YOLO11m (640x640 .bin
from the rdk_model_zoo). Any detection whose box bottom reaches the lower
part of the view is "near"; the robot steers for the widest horizontal gap
between near obstacles.

Honest limitation vs PIDNet: YOLO only sees its 80 trained classes — walls,
doors, cardboard, table legs etc. are INVISIBLE to it. The TF-Luna forward
range is therefore load-bearing here, not just a backstop. Segmentation sees
"not floor"; YOLO sees "a chair". Best for spaces where obstacles are mostly
people/furniture-type objects.

Timing: YOLO11m is ~52 ms end-to-end per frame (~19 FPS). The front camera is
inferred EVERY control cycle, the two sides alternate — cycle ~10 Hz, each
side refreshed at ~5 Hz.

Control: same FORWARD/TURN/BACKUP state machine as 07, plus gentle in-lane
steering toward the gap centre while driving forward.

Run (wheels ON the ground, area clear):
    sudo python3 scripts/08_yolo_avoid.py --dry-run     # perception only
    sudo python3 scripts/08_yolo_avoid.py               # motors live
Watch  http://<robot-ip>:8080  for detections + gap + decision.
"""
import argparse
import importlib.util
import logging
import os
import sys
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np

ZOO = "/home/sunrise/rdk_model_zoo"
YOLO_DIR = ZOO + "/samples/vision/ultralytics_yolo/runtime/python"
YOLO_BIN = ZOO + "/samples/vision/ultralytics_yolo/model/yolo11m_detect_bayese_640x640_nv12.bin"

# reuse camera/lidar/drive/http plumbing from the PIDNet demo
_spec = importlib.util.spec_from_file_location(
    "pidnet_avoid", os.path.join(os.path.dirname(__file__), "07_pidnet_avoid.py"))
_pid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pid)
CamGrabber, TFLuna, serve_view = _pid.CamGrabber, _pid.TFLuna, _pid.serve_view
load_drive, BY_PATH = _pid.load_drive, _pid.BY_PATH


def load_yolo():
    """Import the model-zoo wrapper without depending on the caller's cwd."""
    sys.path.insert(0, ZOO)         # for `utils.py_utils.*`
    sys.path.insert(0, YOLO_DIR)
    logging.disable(logging.INFO)   # silence per-frame pre/forward/post logs
    from ultralytics_yolo_det import (UltralyticsYOLODetect,
                                      UltralyticsYOLODetectConfig)
    return UltralyticsYOLODetect(
        UltralyticsYOLODetectConfig(model_path=YOLO_BIN, score_thres=0.35))


def near_obstacles(boxes, scores, frame_h, near_y_frac):
    """Boxes whose bottom edge reaches the near field (lower part of view)."""
    if len(boxes) == 0:
        return boxes[:0]
    keep = boxes[:, 3] >= frame_h * near_y_frac
    return boxes[keep]


def widest_gap(near_boxes, frame_w, margin_frac=0.05):
    """-> (gap_left, gap_right) of the widest x-interval free of near boxes."""
    m = frame_w * margin_frac
    blocked = sorted((max(0.0, b[0] - m), min(frame_w, b[2] + m))
                     for b in near_boxes)
    gaps, x = [], 0.0
    for lo, hi in blocked:
        if lo > x:
            gaps.append((x, lo))
        x = max(x, hi)
    if x < frame_w:
        gaps.append((x, frame_w))
    if not gaps:
        return 0.0, 0.0
    return max(gaps, key=lambda g: g[1] - g[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="perception only, no motors")
    ap.add_argument("--duty", type=float, default=45, help="forward duty %%")
    ap.add_argument("--turn-duty", type=float, default=50, help="spin duty %%")
    ap.add_argument("--stop-cm", type=float, default=30, help="TF-Luna hard stop")
    ap.add_argument("--min-gap", type=float, default=0.35,
                    help="min free gap as fraction of frame width to drive")
    ap.add_argument("--near-y", type=float, default=0.55,
                    help="box bottom below this frame-height fraction = near")
    ap.add_argument("--steer-k", type=float, default=0.5,
                    help="in-lane steering gain toward the gap centre")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    det = load_yolo()
    luna = TFLuna()
    cams = {
        "front": CamGrabber("front", BY_PATH.format("1.1"), "MJPG", (640, 480)),
        "left":  CamGrabber("left",  BY_PATH.format("1.3"), "YUYV", (320, 240)),
        "right": CamGrabber("right", BY_PATH.format("1.4"), "YUYV", (320, 240)),
    }
    for c in cams.values():
        c.start()
    drive = None if args.dry_run else load_drive()
    state = {"view": None}
    serve_view(state, args.port)
    print("warming up cameras ...")
    time.sleep(2.5)

    mode, turn_dir = "FORWARD", "left"
    blocked_n = clear_n = 0
    mode_since = time.time()
    side_toggle = 0
    # cached per-camera results: (boxes, near_boxes, pressure, frame)
    cache = {k: (np.empty((0, 4)), np.empty((0, 4)), 0.0, None) for k in cams}
    t_end = time.time() + args.seconds if args.seconds else None

    def infer(k):
        f = cams[k].latest()
        if f is None:
            cache[k] = (np.empty((0, 4)), np.empty((0, 4)), 1.0, None)  # stale = blocked
            return
        boxes, scores, _cls = det.predict(f)
        near = near_obstacles(boxes, scores, f.shape[0], args.near_y)
        covered = float(sum(b[2] - b[0] for b in near)) / f.shape[1]
        cache[k] = (boxes, near, min(1.0, covered), f)

    try:
        while t_end is None or time.time() < t_end:
            infer("front")
            side = ("left", "right")[side_toggle]     # alternate the sides
            side_toggle ^= 1
            infer(side)

            fboxes, fnear, _p, ffront = cache["front"]
            fw = ffront.shape[1] if ffront is not None else 640
            gap_l, gap_r = widest_gap(fnear, fw)
            gap_frac = (gap_r - gap_l) / fw
            gap_center = (gap_l + gap_r) / 2.0 / fw    # 0..1

            dist = luna.read_cm()
            lidar_block = dist is not None and dist < args.stop_cm
            front_ok = (gap_frac >= args.min_gap and ffront is not None
                        and not lidar_block)

            now = time.time()
            if mode == "FORWARD":
                blocked_n = 0 if front_ok else blocked_n + 1
                if blocked_n >= 2:                     # ~10 Hz loop: 2 cycles
                    turn_dir = ("left" if cache["left"][2] <= cache["right"][2]
                                else "right")
                    mode, mode_since, clear_n = "TURN", now, 0
            elif mode == "TURN":
                clear_n = clear_n + 1 if front_ok and gap_frac >= args.min_gap + 0.1 \
                    else 0
                if clear_n >= 2:
                    mode, mode_since, blocked_n = "FORWARD", now, 0
                elif now - mode_since > 4.0:
                    mode, mode_since = "BACKUP", now
            elif mode == "BACKUP":
                if now - mode_since > 0.7:
                    turn_dir = "right" if turn_dir == "left" else "left"
                    mode, mode_since, clear_n = "TURN", now, 0

            if drive:
                if mode == "FORWARD":
                    steer = max(-1.0, min(1.0, (gap_center - 0.5) * 2 * args.steer_k))
                    drive.set(args.duty * (1 + steer), args.duty * (1 - steer))
                elif mode == "TURN":
                    (drive.spin_left if turn_dir == "left"
                     else drive.spin_right)(args.turn_duty)
                else:
                    drive.backward(args.duty)

            # ---- live view ----
            tiles = []
            for k in ("left", "front", "right"):
                boxes, near, pressure, f = cache[k]
                if f is None:
                    f = np.zeros((240, 320, 3), np.uint8)
                t = cv2.resize(f, (320, 240))
                sx, sy = 320 / f.shape[1], 240 / f.shape[0]
                for b in boxes:
                    is_near = any(np.array_equal(b, nb) for nb in near)
                    cv2.rectangle(t, (int(b[0] * sx), int(b[1] * sy)),
                                  (int(b[2] * sx), int(b[3] * sy)),
                                  (0, 0, 255) if is_near else (0, 200, 255), 2)
                if k == "front":
                    cv2.line(t, (int(gap_l * sx), 235), (int(gap_r * sx), 235),
                             (80, 255, 80), 6)
                cv2.putText(t, f"{k} block={pressure:.2f}", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                tiles.append(t)
            grid = cv2.hconcat(tiles)
            banner = (f"{mode}{'/' + turn_dir if mode != 'FORWARD' else ''}  "
                      f"gap={gap_frac:.2f}@{gap_center:.2f}  "
                      f"lidar={'%.0fcm' % dist if dist else 'n/a'}"
                      f"{' STOP' if lidar_block else ''}"
                      f"{'  [DRY RUN]' if args.dry_run else ''}")
            cv2.putText(grid, banner, (8, grid.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            state["view"] = grid
    except KeyboardInterrupt:
        pass
    finally:
        if drive:
            drive.close()
            print("\nmotors released")
        for c in cams.values():
            c.stop_evt.set()
        for c in cams.values():
            c.join(timeout=2.0)


if __name__ == "__main__":
    main()
