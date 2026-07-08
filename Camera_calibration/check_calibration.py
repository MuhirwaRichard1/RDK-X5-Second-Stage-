import os

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import glob
import sys

import cv2
import numpy as np
import yaml

"""
Headless check of a finished calibration: undistorts the calibration images
and writes original | undistorted side-by-side comparisons to undistort_check/.
If the calibration is good, straight lines (chessboard edges, door frames)
that look curved on the left should look straight on the right.

Usage: python3 check_calibration.py [calibration_yaml] [image_dir]
e.g.   python3 check_calibration.py calibration_front.yaml calib_images_front
"""

CALIB_FILE = sys.argv[1] if len(sys.argv) > 1 else "calibration.yaml"
IMAGE_DIR = sys.argv[2] if len(sys.argv) > 2 else "calib_images_front"
# One subfolder per image set so checking another camera doesn't overwrite
OUT_DIR = os.path.join("undistort_check", os.path.basename(IMAGE_DIR.rstrip("/")))

with open(CALIB_FILE) as f:
    calib = yaml.safe_load(f)

mtx = np.array(calib["camera_matrix"])
dist = np.array(calib["dist_coeff"])

images = sorted(glob.glob(f"{IMAGE_DIR}/*.jpg"))
if not images:
    sys.exit(f"No images in {IMAGE_DIR}/")

os.makedirs(OUT_DIR, exist_ok=True)

for fname in images:
    img = cv2.imread(fname)
    h, w = img.shape[:2]

    # alpha=0 crops to valid pixels only. Don't use alpha=1 on these wide
    # lenses: it keeps edge regions where the distortion model extrapolates
    # wildly, which looks broken even when the calibration is good.
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
    undistorted = cv2.undistort(img, mtx, dist, None, new_mtx)

    side = np.hstack([img, undistorted])
    cv2.putText(side, "original", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(side, "undistorted", (w + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    out = os.path.join(OUT_DIR, os.path.basename(fname))
    cv2.imwrite(out, side)
    print(f"Wrote {out}")

print(f"\nOpen the images in {os.path.abspath(OUT_DIR)} to judge the calibration.")
