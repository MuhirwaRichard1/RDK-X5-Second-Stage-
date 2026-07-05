#!/usr/bin/env python3
"""
tri_cam_grid — combine the NavBot's 3 USB cameras into one live view.

Layout is a simple 1x3 grid  [ LEFT | FRONT | RIGHT ]  — deliberately NOT
panoramic stitching: feature matching + warping would burn CPU the X5 needs
for the AI stack, and the cameras face different directions so there is
little overlap to stitch anyway. hconcat is essentially free.

Bandwidth note: all 3 cams share one USB-2 (480 Mbps) bus, and the two side
cameras (1e45:8022 clones) hog the whole bus for ANY MJPEG mode, so only the
front camera uses MJPG; the sides capture YUYV 320x240 (needs uvcvideo
quirks=128 — see three_cam.launch.py and /etc/modprobe.d/uvcvideo-navbot.conf).

View it (robot is headless — watch from your laptop browser):
    python3 scripts/tri_cam_grid.py                 # serves http://<robot-ip>:8080
    python3 scripts/tri_cam_grid.py --save out.jpg  # one composite snapshot
    python3 scripts/tri_cam_grid.py --display       # local window (needs DISPLAY)

Devices default to /dev/v4l/by-path/* symlinks, which are stable per PHYSICAL
port (front = port 1.1, left = 1.3, right = 1.4); /dev/videoN order is not.
Override if a camera moves:
    python3 scripts/tri_cam_grid.py --left <dev> --front <dev> --right <dev>
"""

import argparse
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# quiet the per-retry V4L2 "can't open" warnings while a camera is unplugged
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np

TILE_W, TILE_H = 640, 480          # per-camera tile size in the grid
JPEG_QUALITY = 70                  # for the MJPEG http stream


class CamGrabber(threading.Thread):
    """Continuously grabs the newest frame so readers never see stale buffers."""

    def __init__(self, name, device, fps, fourcc="YUYV", size=(320, 240)):
        super().__init__(daemon=True)
        self.name_ = name
        self.device = device
        self.fps = fps
        self.fourcc = fourcc
        self.size = size
        self.frame = None
        self.lock = threading.Lock()
        self.ok = False
        self.measured_fps = 0.0

    def _open(self):
        # OpenCV's V4L2 backend wants an index, not a path; resolve symlinks
        # (/dev/v4l/by-path/*) down to /dev/videoN first.
        m = re.match(r"/dev/video(\d+)$", os.path.realpath(self.device))
        cap = cv2.VideoCapture(int(m.group(1)) if m else self.device,
                               cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        return cap

    def run(self):
        cap = self._open()
        if not cap.isOpened():
            print(f"[{self.name_}] cannot open {self.device} (will keep retrying)")
        n, t0 = 0, time.time()
        while True:
            ret, frame = cap.read()
            if not ret:
                self.ok = False
                time.sleep(1.0)         # not present / unplugged: retry forever
                cap.release()
                cap = self._open()
                n, t0 = 0, time.time()
                continue
            self.ok = True
            with self.lock:
                self.frame = frame
            n += 1
            if n >= 30:
                self.measured_fps = n / (time.time() - t0)
                n, t0 = 0, time.time()

    def tile(self):
        """Return a labelled TILE_W x TILE_H image (placeholder if no signal)."""
        with self.lock:
            f = None if self.frame is None else self.frame.copy()
        if f is None or not self.ok:
            f = np.full((TILE_H, TILE_W, 3), 40, np.uint8)
            cv2.putText(f, f"{self.name_}: NO SIGNAL", (40, TILE_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return f
        if f.shape[1] != TILE_W or f.shape[0] != TILE_H:
            f = cv2.resize(f, (TILE_W, TILE_H), interpolation=cv2.INTER_NEAREST)
        cv2.putText(f, f"{self.name_} {self.measured_fps:.0f}fps", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return f


def make_grid(cams):
    return cv2.hconcat([c.tile() for c in cams])


# --------------------------- MJPEG over HTTP --------------------------- #
def serve_mjpeg(cams, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):      # silence per-request spam
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(b'<html><body style="margin:0;background:#222">'
                                 b'<img src="/stream" style="width:100%">'
                                 b"</body></html>")
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    grid = make_grid(cams)
                    ok, jpg = cv2.imencode(
                        ".jpg", grid, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if ok:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n")
                        self.wfile.write(jpg.tobytes())
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass                    # viewer closed the tab

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"grid view: http://<robot-ip>:{port}  (Ctrl-C to stop)")
    srv.serve_forever()


def main():
    by_path = "/dev/v4l/by-path/platform-xhci-hcd.2.auto-usb-0:{}:1.0-video-index0"
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", default=by_path.format("1.3"))
    ap.add_argument("--front", default=by_path.format("1.1"))
    ap.add_argument("--right", default=by_path.format("1.4"))
    ap.add_argument("--fps", type=int, default=30,
                    help="requested fps (the cameras only advertise 30)")
    ap.add_argument("--port", type=int, default=8080, help="MJPEG http port")
    ap.add_argument("--save", metavar="FILE", help="write one composite jpg and exit")
    ap.add_argument("--display", action="store_true", help="local cv2 window")
    args = ap.parse_args()

    # front (0bdc:8088) does honest MJPEG; the 8022 side clones must use YUYV
    cams = [CamGrabber("LEFT", args.left, args.fps),
            CamGrabber("FRONT", args.front, args.fps,
                       fourcc="MJPG", size=(640, 480)),
            CamGrabber("RIGHT", args.right, args.fps)]
    for c in cams:
        c.start()
    time.sleep(2.0)                     # let cameras warm up

    if args.save:
        cv2.imwrite(args.save, make_grid(cams))
        print(f"saved {args.save}")
        return
    if args.display:
        while True:
            cv2.imshow("tri-cam grid", make_grid(cams))
            if cv2.waitKey(30) & 0xFF in (27, ord("q")):
                break
        cv2.destroyAllWindows()
        return
    serve_mjpeg(cams, args.port)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
