import os

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import select
import socket
import sys
import termios
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

"""
Collect images for camera calibration using a chessboard pattern.

The robot has no display (and cv2 is the headless build), so the live
preview is served as an MJPEG stream instead of cv2.imshow:

    1. Run this script over SSH:  python3 collecting_chessboard_images.py [cam_index] [save_dir]
    2. Open http://<robot-ip>:8090 in a browser on your PC (URL is printed on start)
    3. Keys are read from THIS terminal: SPACE or s = save image, q = quit
"""

# Directory to save images
SAVE_DIR = "calib_images_right"
# Number of images, use at least 15-20 images
NUM_IMAGES = 30
# How many inner corners per chessboard row and column, not squares (IMPORTANT !!!)
CHESSBOARD_SIZE = (9, 6)
# 0 = default webcam, override with the first CLI argument
CAM_INDEX = 2
# Port for the MJPEG preview stream (8080 is used by the robot agent)
PORT = 8090
# Detect corners on a downscaled frame so the preview stays fluid; the
# full-resolution frame is what gets saved.
DETECT_SCALE = 0.5

if len(sys.argv) > 1:
    CAM_INDEX = int(sys.argv[1])
if len(sys.argv) > 2:
    SAVE_DIR = sys.argv[2]

latest_jpeg = None
jpeg_lock = threading.Lock()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with jpeg_lock:
                    buf = latest_jpeg
                if buf is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(buf)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


def robot_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "<robot-ip>"
    finally:
        s.close()


os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    sys.exit(f"Cannot open camera {CAM_INDEX}")

server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
threading.Thread(target=server.serve_forever, daemon=True).start()

print(f"Live preview: http://{robot_ip()}:{PORT}")
print(f"Saving to {os.path.abspath(SAVE_DIR)}")
print("In this terminal: SPACE or s = save image, q = quit\n")

count = 0
old_tty = None
if sys.stdin.isatty():
    old_tty = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

try:
    while count < NUM_IMAGES:
        ret, frame = cap.read()
        if not ret:
            print("Camera stopped returning frames.")
            break

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, None, fx=DETECT_SCALE, fy=DETECT_SCALE)

        # FAST_CHECK keeps the preview responsive; the calibration script
        # re-detects corners on the saved full-resolution images anyway.
        found, corners = cv2.findChessboardCorners(small, CHESSBOARD_SIZE,
                                                   cv2.CALIB_CB_FAST_CHECK)

        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE,
                                      corners / DETECT_SCALE, found)
            cv2.putText(display, "Chessboard found - press SPACE to save", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(display, f"saved {count}/{NUM_IMAGES}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        ok, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with jpeg_lock:
                latest_jpeg = buf.tobytes()

        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key in (" ", "s") and found:
                filename = os.path.join(SAVE_DIR, f"img_{count:02d}.jpg")
                cv2.imwrite(filename, frame)
                print(f"Saved {filename}")
                count += 1
            elif key == "q":
                break
finally:
    if old_tty is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
    cap.release()
    server.shutdown()

print(f"\nDone: {count} images in {SAVE_DIR}")
