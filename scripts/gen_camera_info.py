#!/usr/bin/env python3
"""
gen_camera_info.py — convert the calibrator's output (Camera_calibration/
calibration_{front,left,right}.yaml, all shot at 640x480) into the ROS
camera_info YAML format that `hobot_usb_cam`'s camera_calibration_file_path
(camera_info_manager) loads, writing config/camera_{front,left,right}.yaml.

Resolutions the SLAM stack actually streams (see three_cam.launch.py):
  front : 640x480  -> intrinsics used as-is (calibrated at 640x480)
  left  : 320x240  -> intrinsics scaled x0.5 (same 4:3 aspect => pure scale)
  right : 320x240  -> intrinsics scaled x0.5

Distortion coefficients (k1,k2,p1,p2,k3) are dimensionless and resolution
independent, so they are copied unchanged when scaling. Monocular cameras:
rectification = identity, projection = [K | 0].
"""

import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CALIB_DIR = os.path.join(ROOT, "Camera_calibration")
OUT_DIR = os.path.join(ROOT, "config")

# (name, source calib file, output width, output height, scale applied to K)
CAMS = [
    ("cam_front", "calibration_front.yaml", 640, 480, 1.0),
    ("cam_left",  "calibration_left.yaml",  320, 240, 0.5),
    ("cam_right", "calibration_right.yaml", 320, 240, 0.5),
]


def load_calib(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = [row[:] for row in d["camera_matrix"]]      # 3x3
    dist = list(d["dist_coeff"][0])                 # 5
    return K, dist, d.get("reprojection_error")


def scale_K(K, s):
    # fx, fy, cx, cy scale with pixel resolution; skew (0) and the homogeneous
    # row are unchanged.
    return [
        [K[0][0] * s, K[0][1],     K[0][2] * s],
        [K[1][0],     K[1][1] * s, K[1][2] * s],
        [0.0,         0.0,         1.0],
    ]


def cam_info_yaml(name, w, h, K, dist):
    fx, _, cx = K[0]
    _, fy, cy = K[1]
    return {
        "image_width": w,
        "image_height": h,
        "camera_name": name,
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": list(dist)},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0]},
        # monocular: projection = [K | 0]
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [fx, 0.0, cx, 0.0,
                                       0.0, fy, cy, 0.0,
                                       0.0, 0.0, 1.0, 0.0]},
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, src, w, h, s in CAMS:
        K, dist, rms = load_calib(os.path.join(CALIB_DIR, src))
        Ks = scale_K(K, s) if s != 1.0 else K
        info = cam_info_yaml(name, w, h, Ks, dist)
        out = os.path.join(OUT_DIR, name.replace("cam_", "camera_") + ".yaml")
        with open(out, "w") as f:
            yaml.safe_dump(info, f, sort_keys=False, default_flow_style=None)
        print(f"{name}: {w}x{h} fx={Ks[0][0]:.2f} fy={Ks[1][1]:.2f} "
              f"cx={Ks[0][2]:.2f} cy={Ks[1][2]:.2f} (src RMS={rms:.3f}px) -> {out}")


if __name__ == "__main__":
    main()
