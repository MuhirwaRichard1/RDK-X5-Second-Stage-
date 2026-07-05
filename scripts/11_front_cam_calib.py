#!/usr/bin/env python3
"""Step 11 - front camera intrinsics calibration (VIO SLAM plan, Phase 1.1).

Live chessboard capture + OpenCV calibration in one run:
  1. Print a chessboard (default 9x6 INNER corners, e.g. a 10x7-square board),
     tape it FLAT to something rigid, measure one square in mm.
  2. Run this script, open http://<robot-ip>:8080 and move the board through
     the view: near/far, all four corners, strong tilts. A frame is captured
     automatically (green flash) when corners are found and the board has
     moved since the last capture.
  3. After --frames captures the script calibrates, prints the RMS
     reprojection error and writes a ROS camera_info YAML.

Pass if:  RMS reprojection error < 0.5 px (re-run with better coverage if not).

Run:      python3 scripts/11_front_cam_calib.py --square-mm 24
Output:   config/camera_front.yaml  (+ raw frames in config/calib_frames/)
"""
import argparse
import importlib.util
import os
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np

_spec = importlib.util.spec_from_file_location(
    "pidnet_avoid", os.path.join(os.path.dirname(__file__), "07_pidnet_avoid.py"))
_pid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pid)
CamGrabber, serve_view, BY_PATH = _pid.CamGrabber, _pid.serve_view, _pid.BY_PATH

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def write_camera_info_yaml(path, name, size, K, D, rms):
    w, h = size
    P = np.zeros((3, 4))
    P[:3, :3] = K
    fmt = lambda a: "[" + ", ".join(f"{v:.6f}" for v in np.asarray(a).ravel()) + "]"
    with open(path, "w") as f:
        f.write(f"""# OpenCV calibration {time.strftime('%Y-%m-%d %H:%M')}, RMS reprojection {rms:.3f} px
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: {fmt(K)}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: {fmt(D[:5])}
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: {fmt(P)}
""")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square-mm", type=float, default=24.0)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm / 1000.0                 # metres

    frames_dir = os.path.join(REPO, "config", "calib_frames")
    os.makedirs(frames_dir, exist_ok=True)

    cam = CamGrabber("front", BY_PATH.format("1.1"), "MJPG",
                     (args.width, args.height))
    cam.start()
    state = {"view": None}
    serve_view(state, args.port)
    time.sleep(2.0)
    print(f"capturing {args.frames} boards of {pattern} inner corners — "
          f"watch :{args.port} and move the board around")

    obj_pts, img_pts = [], []
    last_corners, flash = None, 0
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    try:
        while len(img_pts) < args.frames:
            f = cam.latest()
            if f is None:
                time.sleep(0.1)
                continue
            gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
            vis = f.copy()
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
                cv2.drawChessboardCorners(vis, pattern, corners, found)
                moved = (last_corners is None or
                         np.abs(corners - last_corners).mean() > 15.0)
                if moved:
                    obj_pts.append(objp)
                    img_pts.append(corners)
                    last_corners = corners
                    flash = 3
                    cv2.imwrite(os.path.join(
                        frames_dir, f"calib_{len(img_pts):02d}.jpg"), f)
                    print(f"  captured {len(img_pts)}/{args.frames}")
            if flash > 0:
                cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, vis.shape[0] - 1),
                              (0, 255, 0), 12)
                flash -= 1
            cv2.putText(vis, f"{len(img_pts)}/{args.frames} boards", (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            state["view"] = vis
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\ninterrupted — calibrating with what we have")
    finally:
        cam.stop_evt.set()
        cam.join(timeout=2.0)

    if len(img_pts) < 10:
        print(f"only {len(img_pts)} boards — too few, aborting")
        return 1

    print(f"calibrating on {len(img_pts)} boards ...")
    rms, K, D, _r, _t = cv2.calibrateCamera(
        obj_pts, img_pts, (args.width, args.height), None, None)
    out = os.path.join(REPO, "config", "camera_front.yaml")
    write_camera_info_yaml(out, "cam_front", (args.width, args.height),
                           K, D.ravel(), rms)
    print(f"RMS reprojection error: {rms:.3f} px "
          f"({'PASS' if rms < 0.5 else 'HIGH — re-run with better coverage'})")
    print(f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
