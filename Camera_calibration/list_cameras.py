import os

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import math

import cv2
import numpy as np

"""
Headless camera lister for the robot (no display / opencv-python-headless).
Probes /dev/video* devices, prints what it finds and saves a labelled
snapshot grid to cameras_preview.jpg so you can check which index is
which camera from your PC.
"""

MAX_CAMERAS = 10
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
PREVIEW_FILE = "cameras_preview.jpg"


def device_name(i):
    try:
        with open(f"/sys/class/video4linux/video{i}/name") as f:
            return f.read().strip()
    except OSError:
        return "unknown device"


print("Searching for cameras...\n")

frames = []
available = []

for i in range(MAX_CAMERAS):
    if not os.path.exists(f"/dev/video{i}"):
        continue

    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)

    if not cap.isOpened():
        cap.release()
        continue

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print(f"[{i}] {device_name(i)} - opens but returns no frames (check cable / device busy)")
        continue

    h, w = frame.shape[:2]
    print(f"[{i}] {device_name(i)} ({w}x{h})")
    available.append(i)

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    cv2.putText(frame,
                f"Camera {i}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                2)
    frames.append(frame)

if not available:
    print("No working cameras found.")
    raise SystemExit(1)

cols = math.ceil(math.sqrt(len(frames)))
rows = math.ceil(len(frames) / cols)

blank = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
while len(frames) < rows * cols:
    frames.append(blank.copy())

grid = np.vstack([np.hstack(frames[r * cols:(r + 1) * cols]) for r in range(rows)])
cv2.imwrite(PREVIEW_FILE, grid)

print(f"\nAvailable camera indices: {available}")
print(f"Snapshot grid saved to {os.path.abspath(PREVIEW_FILE)}")
