#!/usr/bin/env python3
"""Step 10 - live dry-run preview for the Depth Anything V2 BPU builds.

Streams the front camera through ONE compiled depth .bin and serves
[camera | colorized depth] side-by-side over HTTP, with the measured BPU
latency in the banner — for judging the four ViT-S variants against each
other (~/Desktop/RDK/model_output_vits*).

Depth Anything outputs RELATIVE inverse depth (bright = close, dark = far);
values are normalized per frame for display.

Run one model:
    sudo python3 scripts/10_depth_preview.py --model vits [--seconds 30]
    (--model accepts vits | vits392 | vitsopt | vitsopt2 or a .bin path)
Watch  http://<robot-ip>:8080
"""
import argparse
import importlib.util
import os
import time

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

RDK = "/home/sunrise/Desktop/RDK"
MODELS = {
    "vits":     RDK + "/model_output_vits/depth_anything_v2_vits_518.bin",
    "vits392":  RDK + "/model_output_vits392/depth_anything_v2_vits392.bin",
    "vitsopt":  RDK + "/model_output_vitsopt/depth_anything_v2_vitsopt_518.bin",
    "vitsopt2": RDK + "/model_output_vitsopt2/depth_anything_v2_vitsopt2_518.bin",
}

_spec = importlib.util.spec_from_file_location(
    "pidnet_avoid", os.path.join(os.path.dirname(__file__), "07_pidnet_avoid.py"))
_pid = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pid)
CamGrabber, serve_view, BY_PATH = _pid.CamGrabber, _pid.serve_view, _pid.BY_PATH
bgr2nv12 = _pid.bgr2nv12


class DepthModel:
    def __init__(self, path):
        self.model = dnn.load(path)[0]
        shape = self.model.inputs[0].properties.shape
        self.size = shape[2]                     # square input, H == W
        self.name = os.path.basename(path)

    def infer(self, bgr):
        """-> (depth map size x size float32, forward ms)."""
        resized = cv2.resize(bgr, (self.size, self.size),
                             interpolation=cv2.INTER_LINEAR)
        nv12 = bgr2nv12(resized)
        t0 = time.monotonic()
        out = self.model.forward(nv12)
        ms = (time.monotonic() - t0) * 1000
        return out[0].buffer.reshape(self.size, self.size), ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="vits",
                    help="vits | vits392 | vitsopt | vitsopt2 | /path/to.bin")
    ap.add_argument("--seconds", type=float, default=0, help="auto-stop after N s")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    path = MODELS.get(args.model, args.model)
    depth_net = DepthModel(path)
    print(f"model: {depth_net.name}  input {depth_net.size}x{depth_net.size}")

    cam = CamGrabber("front", BY_PATH.format("1.1"), "MJPG", (640, 480))
    cam.start()
    state = {"view": None}
    serve_view(state, args.port)
    time.sleep(2.0)

    ms_avg, n = 0.0, 0
    t_end = time.time() + args.seconds if args.seconds else None
    try:
        while t_end is None or time.time() < t_end:
            f = cam.latest()
            if f is None:
                time.sleep(0.1)
                continue
            depth, ms = depth_net.infer(f)
            n += 1
            ms_avg += (ms - ms_avg) / n         # running mean

            d = (depth - depth.min()) / (depth.max() - depth.min() + 1e-8)
            vis = cv2.applyColorMap((d * 255).astype(np.uint8),
                                    cv2.COLORMAP_INFERNO)
            vis = cv2.resize(vis, (426, 320))
            left = cv2.resize(f, (426, 320))
            grid = cv2.hconcat([left, vis])
            cv2.putText(grid, f"{depth_net.name}  {ms:.0f}ms (avg {ms_avg:.0f}ms,"
                        f" {1000/ms_avg:.1f} FPS)", (8, grid.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            state["view"] = grid
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop_evt.set()
        cam.join(timeout=2.0)
        if n:
            print(f"{depth_net.name}: {n} frames, avg forward {ms_avg:.1f} ms "
                  f"({1000/ms_avg:.1f} FPS)")


if __name__ == "__main__":
    main()
