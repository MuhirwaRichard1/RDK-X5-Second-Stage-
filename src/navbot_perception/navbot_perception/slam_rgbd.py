#!/usr/bin/env python3
"""
slam_rgbd — RGB-D front-end for the RTAB-Map SLAM track (Track A).

The robot has no depth camera, so we synthesize one for the front camera:

  front JPEG ─decode─► rectify (camera_front.yaml maps) ─► Depth Anything V2
  (BPU, relative inverse depth) ─► scale to METRES with the TF-Luna forward
  range ─► publish a registered RGB-D pair RTAB-Map can consume.

Published (all sharing the SOURCE FRAME's timestamp so rgbd_odometry syncs
them exactly, frame_id = `cam_front`):
  ~/rgb/image_rect     sensor_msgs/Image  bgr8   640x480, undistorted
  ~/rgb/camera_info    sensor_msgs/CameraInfo    K from calib, D=0 (rectified)
  ~/depth/image_rect   sensor_msgs/Image  32FC1  metres, 0 = no-depth, registered

Metric scaling (single-point, per the vio_slam_plan): Depth Anything outputs an
affine-invariant inverse depth `raw` (larger = closer). Assuming the shift is
negligible, metric depth is  d(u,v) = range * raw_center / raw(u,v), which pins
the boresight pixel to the measured TF-Luna range and propagates relative
geometry outward. This is the weak link of Track A (crude far-field scale, per-
frame scale jitter) — mitigated by EMA-smoothing the range anchor and holding
the last good value across TF-Luna dropouts. Depth beyond the TF-Luna window
[min_range, max_range] is emitted as 0 (RTAB-Map treats 0 as "no measurement").

BPU note: this loads its own Depth Anything instance. Do NOT run `depth_bpu`
(the console grid) at the same time — one ION pool cannot hold two copies. Only
launched by the SLAM bringup, which drops the console depth grid while mapping.
"""

import array
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Range

from hobot_dnn import pyeasy_dnn as dnn

from .imgproc import bgr2nv12, decode

# vits392 (smaller input than vits_518) — lighter on the 320 MB ION pool and
# faster, so depth FPS (=> odometry rate) is higher. Override via the model_bin
# param to trade accuracy for the 518 model when the BPU is dedicated to SLAM.
MODEL_BIN = "/home/sunrise/Desktop/RDK/model_output_vits392/depth_anything_v2_vits392.bin"
STALE_S = 0.6


class SlamRgbd(Node):
    def __init__(self):
        super().__init__("slam_rgbd")

        self.declare_parameter("front_topic", "/cam_front/image_raw")
        self.declare_parameter("range_topic", "/range_forward")
        self.declare_parameter("calib_file",
                               "/home/sunrise/rdk-x5-navbot/config/camera_front.yaml")
        self.declare_parameter("model_bin", MODEL_BIN)
        self.declare_parameter("frame_id", "cam_front")
        self.declare_parameter("rate_hz", 6.0)
        # boresight: pixel the TF-Luna beam hits; default = image centre. Sampled
        # over a (2*win+1)^2 median window for robustness.
        self.declare_parameter("boresight_x", -1)   # -1 => image centre
        self.declare_parameter("boresight_y", -1)
        self.declare_parameter("boresight_win", 6)
        self.declare_parameter("range_ema", 0.4)    # EMA weight on new range
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame_id = g("frame_id")
        self.bwin = int(g("boresight_win"))
        self.range_ema = float(g("range_ema"))

        self._load_rectify_maps(g("calib_file"))
        self._load_model(g("model_bin"))

        self.frame = None
        self.stamp = 0.0
        self.range_m = float("nan")     # smoothed TF-Luna range (metres)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, g("front_topic"),
                                 self._store, qos)
        self.create_subscription(Range, g("range_topic"), self._on_range, 10)

        self.pub_rgb = self.create_publisher(Image, "~/rgb/image_rect", 5)
        self.pub_info = self.create_publisher(CameraInfo, "~/rgb/camera_info", 5)
        self.pub_depth = self.create_publisher(Image, "~/depth/image_rect", 5)

        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        self.get_logger().info(
            f"slam_rgbd up: {self.w}x{self.h} rect, model {self.size}x{self.size}, "
            f"boresight=({self.bx},{self.by})")

    # ------------------------------------------------------------------ #
    def _load_rectify_maps(self, calib_file):
        with open(calib_file) as f:
            c = yaml.safe_load(f)
        self.w = int(c["image_width"])
        self.h = int(c["image_height"])
        K = np.array(c["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(c["distortion_coefficients"]["data"], dtype=np.float64)
        # Rectify onto the SAME K (no crop) so the rectified camera_info is just
        # K with zero distortion — the P matrix rgbd_odometry needs.
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, K, (self.w, self.h), cv2.CV_16SC2)
        self.K = K
        # boresight pixel (default centre)
        bx = self.get_parameter("boresight_x").value
        by = self.get_parameter("boresight_y").value
        self.bx = self.w // 2 if bx < 0 else int(bx)
        self.by = self.h // 2 if by < 0 else int(by)

        info = CameraInfo()
        info.width, info.height = self.w, self.h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]              # rectified
        info.k = K.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0,
                  0.0, K[1, 1], K[1, 2], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info = info
        self.min_range, self.max_range = 0.1, 8.0

    def _load_model(self, model_bin):
        try:
            self.model = dnn.load(model_bin)[0]
        except Exception as exc:
            # Same failure mode as depth_bpu: BPU ION pool can't fit the model.
            self.get_logger().error(
                f"Depth Anything load failed ({exc!r}) — is another BPU model "
                f"(depth_bpu/detection_bpu) holding ION? slam_rgbd cannot run.")
            raise
        self.size = self.model.inputs[0].properties.shape[2]

    # ------------------------------------------------------------------ #
    def _store(self, msg):
        self.frame = msg
        self.stamp = time.time()

    def _on_range(self, msg):
        r = msg.range
        if not np.isfinite(r) or r < self.min_range or r > self.max_range:
            return                                       # hold last good range
        self.range_m = r if np.isnan(self.range_m) else \
            (1 - self.range_ema) * self.range_m + self.range_ema * r

    def _metric_depth(self, raw):
        """raw (HxW, relative inverse depth) + TF-Luna range -> metres, 32FC1."""
        raw = np.maximum(raw, 1e-3)                      # avoid /0 at far field
        rc = float(np.median(
            raw[max(0, self.by - self.bwin):self.by + self.bwin + 1,
                max(0, self.bx - self.bwin):self.bx + self.bwin + 1]))
        if rc <= 1e-3 or np.isnan(self.range_m):
            return None                                  # can't anchor this frame
        depth = (self.range_m * rc) / raw                # metres
        depth[(depth < self.min_range) | (depth > self.max_range)] = 0.0
        return depth.astype(np.float32)

    def _tick(self):
        if self.frame is None or time.time() - self.stamp > STALE_S:
            return
        msg = self.frame
        bgr = decode(msg)
        if bgr is None:
            return
        if bgr.shape[1] != self.w or bgr.shape[0] != self.h:
            bgr = cv2.resize(bgr, (self.w, self.h))
        rect = cv2.remap(bgr, self.map1, self.map2, cv2.INTER_LINEAR)

        resized = cv2.resize(rect, (self.size, self.size))
        out = self.model.forward(bgr2nv12(resized))
        raw = out[0].buffer.reshape(self.size, self.size).astype(np.float32)
        raw = cv2.resize(raw, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        depth = self._metric_depth(raw)
        if depth is None:
            return

        stamp = msg.header.stamp                         # source-frame time
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.pub_info.publish(self.info)

        rgb = Image()
        rgb.header.stamp = stamp
        rgb.header.frame_id = self.frame_id
        rgb.height, rgb.width = self.h, self.w
        rgb.encoding = "bgr8"
        rgb.step = self.w * 3
        # Wrap in array('B',...): assigning raw bytes to a uint8[] ROS field
        # hits rclpy's per-element validation (~0.9 s for a 640x480x3 image);
        # array('B') takes rclpy's fast C path (~1 ms). This is THE throughput
        # fix — without it a whole tick is ~2.5 s (0.4 Hz) instead of ~0.4 s.
        rgb.data = array.array('B', rect.tobytes())
        self.pub_rgb.publish(rgb)

        dep = Image()
        dep.header.stamp = stamp
        dep.header.frame_id = self.frame_id
        dep.height, dep.width = self.h, self.w
        dep.encoding = "32FC1"
        dep.step = self.w * 4
        dep.data = array.array('B', depth.tobytes())     # fast path — see rgb above
        self.pub_depth.publish(dep)


def main(args=None):
    rclpy.init(args=args)
    node = SlamRgbd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
