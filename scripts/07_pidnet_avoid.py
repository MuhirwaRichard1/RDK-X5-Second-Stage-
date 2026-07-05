#!/usr/bin/env python3
"""Step 7 - tri-camera obstacle avoidance with PIDNet segmentation on the BPU.

Perception: each camera frame -> PIDNet-S 576x768 (Cityscapes, 19 classes,
compiled .bin from ~/Desktop/RDK/model_output_pidnets) -> argmax class map at
1/8 res (72x96) -> "free fraction" = share of drivable pixels (road/sidewalk/
terrain) in the bottom of the view. ~9 ms per forward pass, all 3 cams each
control cycle. A Cityscapes model indoors is approximate: bare floor reads as
"road" (good) but texture-less walls can too — the TF-Luna forward range is a
HARD stop that vision cannot override.

Control (state machine @ ~15 Hz, hysteresis on all transitions):
  FORWARD  drive straight while front free-fraction is high and lidar clear
  TURN     spin toward the side camera seeing more free space; back to
           FORWARD once the front clears, BACKUP if it never does
  BACKUP   short reverse, then turn the other way

Run (wheels ON the ground, area clear):
    sudo python3 scripts/07_pidnet_avoid.py --dry-run    # perception only
    sudo python3 scripts/07_pidnet_avoid.py              # motors live
Watch  http://<robot-ip>:8080  for segmentation overlays + the live decision.

Needs: pwm3 overlay (motors), uvcvideo quirks=128 (3 cams), TF-Luna+I2C5.
"""
import argparse
import importlib.util
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn
from smbus2 import SMBus, i2c_msg

MODEL_BIN = "/home/sunrise/Desktop/RDK/model_output_pidnets/pidnet_s_576x768.bin"
BY_PATH = "/dev/v4l/by-path/platform-xhci-hcd.2.auto-usb-0:{}:1.0-video-index0"
DRIVABLE = (0, 1, 9)            # Cityscapes: road, sidewalk, terrain
TFLUNA_ADDR = 0x10

PALETTE = np.zeros((19, 3), np.uint8)
PALETTE[:] = (60, 20, 220)                     # obstacle-ish: red
for c in DRIVABLE:
    PALETTE[c] = (80, 200, 80)                 # drivable: green


def load_drive():
    """Import the Drive class from 05_drive_functions.py (invalid module name)."""
    path = os.path.join(os.path.dirname(__file__), "05_drive_functions.py")
    spec = importlib.util.spec_from_file_location("drive_functions", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Drive()


def bgr2nv12(bgr):
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(h * w * 3 // 2)
    y = yuv[: h * w]
    uv = yuv[h * w:].reshape(2, h * w // 4).transpose(1, 0).reshape(h * w // 2)
    return np.concatenate([y, uv])


class CamGrabber(threading.Thread):
    def __init__(self, name, device, fourcc, size):
        super().__init__(daemon=True)
        self.name_, self.device = name, device
        self.fourcc, self.size = fourcc, size
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()

    def _open(self):
        dev = os.path.realpath(self.device)
        idx = int(dev[10:]) if dev.startswith("/dev/video") else self.device
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        return cap

    def run(self):
        # threads must be JOINED before interpreter exit: a daemon thread
        # cancelled inside native cap.read() aborts the process
        # ("FATAL: exception not rethrown").
        cap = self._open()
        while not self.stop_evt.is_set():
            ret, frame = cap.read()
            if not ret:
                if self.stop_evt.wait(1.0):
                    break
                cap.release()
                cap = self._open()
                continue
            with self.lock:
                self.frame, self.stamp = frame, time.time()
        cap.release()

    def latest(self, max_age=1.0):
        with self.lock:
            if self.frame is None or time.time() - self.stamp > max_age:
                return None
            return self.frame.copy()


class TFLuna:
    def __init__(self, bus_no=5):
        self.bus = SMBus(bus_no)

    def read_cm(self):
        """-> distance in cm, or None if unavailable/weak signal."""
        try:
            w, r = i2c_msg.write(TFLUNA_ADDR, [0x00]), i2c_msg.read(TFLUNA_ADDR, 4)
            self.bus.i2c_rdwr(w, r)
            d = list(r)
            dist, amp = d[0] | d[1] << 8, d[2] | d[3] << 8
            return dist if amp >= 100 and dist > 0 else None
        except OSError:
            return None


class Segmenter:
    def __init__(self, model_path):
        self.model = dnn.load(model_path)[0]
        _, _, self.h, self.w = self.model.inputs[0].properties.shape  # 576, 768

    def free_fraction(self, bgr, center_only=False):
        """-> (free_frac in bottom ROI, class map 72x96) for one frame."""
        resized = cv2.resize(bgr, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        out = self.model.forward(bgr2nv12(resized))
        logits = out[0].buffer.reshape(19, self.h // 8, self.w // 8)
        classes = np.argmax(logits, axis=0).astype(np.uint8)
        roi = classes[classes.shape[0] * 55 // 100:]        # bottom 45 %
        if center_only:                                      # front: steerable path
            c = roi.shape[1]
            roi = roi[:, c * 20 // 100: c * 80 // 100]
        free = float(np.isin(roi, DRIVABLE).mean())
        return free, classes


def overlay_tile(frame, classes, label, tile=(320, 240)):
    f = cv2.resize(frame, tile)
    mask = cv2.resize(PALETTE[classes], tile, interpolation=cv2.INTER_NEAREST)
    f = cv2.addWeighted(f, 0.6, mask, 0.4, 0)
    cv2.putText(f, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return f


def serve_view(state, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(b'<html><body style="margin:0;background:#111">'
                                 b'<img src="/stream" style="width:100%">'
                                 b"</body></html>")
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    grid = state.get("view")
                    if grid is not None:
                        ok, jpg = cv2.imencode(".jpg", grid,
                                               [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg"
                                             b"\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n"
                                             + jpg.tobytes() + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

    threading.Thread(target=ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever,
                     daemon=True).start()
    print(f"live view: http://<robot-ip>:{port}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="perception only, no motors")
    ap.add_argument("--duty", type=float, default=45, help="forward duty %%")
    ap.add_argument("--turn-duty", type=float, default=50, help="spin duty %%")
    ap.add_argument("--stop-cm", type=float, default=30, help="TF-Luna hard stop")
    ap.add_argument("--free-go", type=float, default=0.60, help="front free frac to drive")
    ap.add_argument("--free-clear", type=float, default=0.70, help="free frac to end a turn")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s (0 = run)")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    seg = Segmenter(MODEL_BIN)
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
    t_end = time.time() + args.seconds if args.seconds else None

    try:
        while t_end is None or time.time() < t_end:
            frames = {k: c.latest() for k, c in cams.items()}
            free, maps = {}, {}
            for k in cams:
                if frames[k] is None:
                    free[k] = 0.0                      # stale camera = blocked
                    maps[k] = np.full((72, 96), 2, np.uint8)
                else:
                    free[k], maps[k] = seg.free_fraction(frames[k],
                                                         center_only=(k == "front"))
            dist = luna.read_cm()
            lidar_block = dist is not None and dist < args.stop_cm
            front_ok = free["front"] >= args.free_go and not lidar_block

            # ---- state machine with 3-cycle hysteresis ----
            now = time.time()
            if mode == "FORWARD":
                blocked_n = 0 if front_ok else blocked_n + 1
                if blocked_n >= 3:
                    turn_dir = "left" if free["left"] >= free["right"] else "right"
                    mode, mode_since, clear_n = "TURN", now, 0
            elif mode == "TURN":
                clear_n = clear_n + 1 if (free["front"] >= args.free_clear
                                          and not lidar_block) else 0
                if clear_n >= 3:
                    mode, mode_since, blocked_n = "FORWARD", now, 0
                elif now - mode_since > 4.0:
                    mode, mode_since = "BACKUP", now
            elif mode == "BACKUP":
                if now - mode_since > 0.7:
                    turn_dir = "right" if turn_dir == "left" else "left"
                    mode, mode_since, clear_n = "TURN", now, 0

            if drive:
                if mode == "FORWARD":
                    drive.forward(args.duty)
                elif mode == "TURN":
                    (drive.spin_left if turn_dir == "left"
                     else drive.spin_right)(args.turn_duty)
                else:
                    drive.backward(args.duty)

            # ---- live view ----
            tiles = []
            for k in ("left", "front", "right"):
                f = frames[k] if frames[k] is not None \
                    else np.zeros((240, 320, 3), np.uint8)
                tiles.append(overlay_tile(f, maps[k], f"{k} free={free[k]:.2f}"))
            grid = cv2.hconcat(tiles)
            banner = (f"{mode}{'/' + turn_dir if mode != 'FORWARD' else ''}  "
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
