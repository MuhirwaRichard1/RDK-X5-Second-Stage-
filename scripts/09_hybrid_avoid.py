#!/usr/bin/env python3
"""Step 9 - hybrid obstacle avoidance: PIDNet floor + YOLO objects, fused.

Fusion (front camera, every cycle):
  * PIDNet-S segmentation -> per-COLUMN drivable fraction over the bottom ROI
    ("is there floor in this direction?")
  * YOLO11m detection     -> near-field boxes veto the columns they cover
    ("is there a known object in this direction?")
  * corridor = widest contiguous run of columns that have floor AND no object.
    Drive toward the corridor centre; if it is narrower than --min-gap, the
    TURN/BACKUP state machine from 07/08 takes over.
  * TF-Luna remains the hard forward stop for anything both nets miss.

Side cameras: PIDNet every cycle (9 ms each) + YOLO alternating (52 ms);
side preference = seg_free * (1 - yolo_coverage).

Cycle budget: front 61 ms + sides 18 ms + one side YOLO 52 ms  ->  ~7.5 Hz.

Run (wheels ON the ground, area clear):
    sudo python3 scripts/09_hybrid_avoid.py --dry-run    # perception only
    sudo python3 scripts/09_hybrid_avoid.py              # motors live
Watch  http://<robot-ip>:8080 : green mask = floor, red boxes = near objects,
green bar = chosen corridor.
"""
import argparse
import importlib.util
import os
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(os.path.dirname(__file__), name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_pid = _load("07_pidnet_avoid")
_yolo = _load("08_yolo_avoid")
CamGrabber, TFLuna, serve_view = _pid.CamGrabber, _pid.TFLuna, _pid.serve_view
load_drive, BY_PATH, PALETTE = _pid.load_drive, _pid.BY_PATH, _pid.PALETTE
Segmenter, DRIVABLE = _pid.Segmenter, _pid.DRIVABLE
load_yolo, near_obstacles = _yolo.load_yolo, _yolo.near_obstacles

ROI_TOP_FRAC = 0.55             # bottom 45 % of the class map is the near field


def column_free(classes):
    """Per-column drivable fraction over the bottom ROI of a 72x96 class map."""
    roi = classes[int(classes.shape[0] * ROI_TOP_FRAC):]
    return np.isin(roi, DRIVABLE).mean(axis=0)          # (96,)


def corridor(col_free, near_boxes, frame_w, floor_thresh):
    """-> (lo, hi) column range [0..1] of the widest drivable+object-free run."""
    ncols = col_free.shape[0]
    good = col_free >= floor_thresh
    for b in near_boxes:                                # YOLO veto
        c0 = int(max(0, b[0]) / frame_w * ncols)
        c1 = int(min(frame_w, b[2]) / frame_w * ncols) + 1
        good[c0:c1] = False
    best_lo = best_hi = run_lo = 0
    for c in range(ncols + 1):
        if c < ncols and good[c]:
            continue
        if c - run_lo > best_hi - best_lo:
            best_lo, best_hi = run_lo, c
        run_lo = c + 1
    return best_lo / ncols, best_hi / ncols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="perception only, no motors")
    ap.add_argument("--duty", type=float, default=45, help="forward duty %%")
    ap.add_argument("--turn-duty", type=float, default=50, help="spin duty %%")
    ap.add_argument("--stop-cm", type=float, default=30, help="TF-Luna hard stop")
    ap.add_argument("--min-gap", type=float, default=0.30,
                    help="min corridor width (fraction of view) to drive")
    ap.add_argument("--floor-thresh", type=float, default=0.55,
                    help="column drivable fraction to count as floor")
    ap.add_argument("--near-y", type=float, default=0.55,
                    help="YOLO box bottom below this height fraction = near")
    ap.add_argument("--steer-k", type=float, default=0.5,
                    help="steering gain toward the corridor centre")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    seg = Segmenter(_pid.MODEL_BIN)
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
    empty = np.empty((0, 4), np.float32)
    # per-cam cache: frame, class map, col_free, yolo boxes, near boxes, side score
    cache = {k: dict(frame=None, classes=np.full((72, 96), 2, np.uint8),
                     col_free=np.zeros(96), boxes=empty, near=empty, score=0.0)
             for k in cams}
    t_end = time.time() + args.seconds if args.seconds else None

    try:
        while t_end is None or time.time() < t_end:
            side = ("left", "right")[side_toggle]      # this cycle's YOLO side
            side_toggle ^= 1

            for k in cams:
                f = cams[k].latest()
                c = cache[k]
                c["frame"] = f
                if f is None:
                    c.update(col_free=np.zeros(96), near=empty, score=0.0)
                    continue
                _free, c["classes"] = seg.free_fraction(f)
                c["col_free"] = column_free(c["classes"])
                if k == "front" or k == side:
                    boxes, scores, _ = det.predict(f)
                    c["boxes"] = boxes
                    c["near"] = near_obstacles(boxes, scores, f.shape[0], args.near_y)
                covered = (float(sum(b[2] - b[0] for b in c["near"])) / f.shape[1]
                           if len(c["near"]) else 0.0)
                c["score"] = float(c["col_free"].mean()) * (1.0 - min(1.0, covered))

            fc = cache["front"]
            fw = fc["frame"].shape[1] if fc["frame"] is not None else 640
            lo, hi = corridor(fc["col_free"], fc["near"], fw, args.floor_thresh)
            gap_frac, gap_center = hi - lo, (lo + hi) / 2.0

            dist = luna.read_cm()
            lidar_block = dist is not None and dist < args.stop_cm
            front_ok = (gap_frac >= args.min_gap and fc["frame"] is not None
                        and not lidar_block)

            now = time.time()
            if mode == "FORWARD":
                blocked_n = 0 if front_ok else blocked_n + 1
                if blocked_n >= 2:
                    turn_dir = ("left" if cache["left"]["score"]
                                >= cache["right"]["score"] else "right")
                    mode, mode_since, clear_n = "TURN", now, 0
            elif mode == "TURN":
                clear_n = clear_n + 1 if (front_ok
                                          and gap_frac >= args.min_gap + 0.1) else 0
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
                c = cache[k]
                f = c["frame"] if c["frame"] is not None \
                    else np.zeros((240, 320, 3), np.uint8)
                t = cv2.resize(f, (320, 240))
                mask = cv2.resize(PALETTE[c["classes"]], (320, 240),
                                  interpolation=cv2.INTER_NEAREST)
                t = cv2.addWeighted(t, 0.65, mask, 0.35, 0)
                sx, sy = 320 / f.shape[1], 240 / f.shape[0]
                for b in c["boxes"]:
                    is_near = any(np.array_equal(b, nb) for nb in c["near"])
                    cv2.rectangle(t, (int(b[0] * sx), int(b[1] * sy)),
                                  (int(b[2] * sx), int(b[3] * sy)),
                                  (0, 0, 255) if is_near else (0, 200, 255), 2)
                if k == "front":
                    cv2.line(t, (int(lo * 320), 235), (int(hi * 320), 235),
                             (80, 255, 80), 6)
                cv2.putText(t, f"{k} score={c['score']:.2f}", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                tiles.append(t)
            grid = cv2.hconcat(tiles)
            banner = (f"{mode}{'/' + turn_dir if mode != 'FORWARD' else ''}  "
                      f"corridor={gap_frac:.2f}@{gap_center:.2f}  "
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
