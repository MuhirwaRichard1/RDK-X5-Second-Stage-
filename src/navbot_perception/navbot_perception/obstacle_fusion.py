#!/usr/bin/env python3
"""
obstacle_fusion — PIDNet ground/obstacle split over 3 cameras -> /obstacles.

Subscribes : /cam_front/image_raw (sensor_msgs/CompressedImage, MJPEG)
             /cam_left/image_raw, /cam_right/image_raw (sensor_msgs/Image)
Publishes  : /obstacles (navbot_msgs/Sectors) at `rate_hz` (default 10 Hz)

Each camera frame runs through PIDNet-S (Cityscapes, ~9 ms on the BPU); the
bottom-ROI per-COLUMN drivable fraction (road/sidewalk/terrain) is projected
into world bearings using the camera's mount angle + HFOV, then accumulated
into `n_sectors` sectors spanning [angle_min, angle_max]. A sector nobody
observed — including any covered by a camera whose frames went stale — is
UNKNOWN, which consumers must treat as BLOCKED (fail-safe, see PROPOSAL).

A Cityscapes model indoors reads bare floor as "road" (good) but can read
texture-less walls as road too — keep the TF-Luna safety_gate downstream.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from hobot_dnn import pyeasy_dnn as dnn
from navbot_msgs.msg import Sectors

MODEL_BIN = "/home/sunrise/Desktop/RDK/model_output_pidnets/pidnet_s_576x768.bin"
DRIVABLE = (0, 1, 9)            # Cityscapes: road, sidewalk, terrain
ROI_TOP_FRAC = 0.55             # bottom 45 % of the class map = near field


def bgr2nv12(bgr):
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(h * w * 3 // 2)
    y = yuv[: h * w]
    uv = yuv[h * w:].reshape(2, h * w // 4).transpose(1, 0).reshape(h * w // 2)
    return np.concatenate([y, uv])


def decode(msg):
    """sensor_msgs Image/CompressedImage -> BGR ndarray (None if unsupported)."""
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    buf = np.frombuffer(msg.data, np.uint8)
    enc = msg.encoding.lower()
    if enc in ("yuyv", "yuv422_yuy2", "yuy2"):
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 2),
                            cv2.COLOR_YUV2BGR_YUY2)
    if enc == "nv12":
        return cv2.cvtColor(buf.reshape(msg.height * 3 // 2, msg.width),
                            cv2.COLOR_YUV2BGR_NV12)
    if enc == "bgr8":
        return buf.reshape(msg.height, msg.width, 3)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3),
                            cv2.COLOR_RGB2BGR)
    return None


class ObstacleFusion(Node):
    def __init__(self):
        super().__init__("obstacle_fusion")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("n_sectors", 24)
        self.declare_parameter("angle_min_deg", -130.0)   # rightmost edge
        self.declare_parameter("angle_max_deg", 130.0)    # leftmost edge
        self.declare_parameter("free_thresh", 0.55)
        self.declare_parameter("stale_s", 0.6)
        # bearing of each camera's optical axis (REP-103: left positive)
        self.declare_parameter("front_topic", "/cam_front/image_raw")
        self.declare_parameter("left_topic", "/cam_left/image_raw")
        self.declare_parameter("right_topic", "/cam_right/image_raw")
        self.declare_parameter("front_axis_deg", 0.0)
        self.declare_parameter("left_axis_deg", 90.0)
        self.declare_parameter("right_axis_deg", -90.0)
        self.declare_parameter("front_hfov_deg", 70.0)
        self.declare_parameter("side_hfov_deg", 70.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.n = int(g("n_sectors"))
        self.a0 = np.radians(g("angle_min_deg"))
        self.a1 = np.radians(g("angle_max_deg"))
        self.free_thresh = g("free_thresh")
        self.stale_s = g("stale_s")

        self.model = dnn.load(MODEL_BIN)[0]
        _, _, self.mh, self.mw = self.model.inputs[0].properties.shape

        self.cams = {
            "front": dict(axis=np.radians(g("front_axis_deg")),
                          hfov=np.radians(g("front_hfov_deg"))),
            "left":  dict(axis=np.radians(g("left_axis_deg")),
                          hfov=np.radians(g("side_hfov_deg"))),
            "right": dict(axis=np.radians(g("right_axis_deg")),
                          hfov=np.radians(g("side_hfov_deg"))),
        }
        for c in self.cams.values():
            c.update(frame=None, stamp=0.0)

        qos = rclpy.qos.QoSProfile(
            depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, g("front_topic"),
                                 lambda m: self._store("front", m), qos)
        self.create_subscription(Image, g("left_topic"),
                                 lambda m: self._store("left", m), qos)
        self.create_subscription(Image, g("right_topic"),
                                 lambda m: self._store("right", m), qos)

        self.pub = self.create_publisher(Sectors, "/obstacles", 10)
        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info(
            f"obstacle_fusion up: {self.n} sectors "
            f"[{g('angle_min_deg'):.0f}..{g('angle_max_deg'):.0f}]deg, "
            f"PIDNet {self.mw}x{self.mh}")

    def _store(self, key, msg):
        self.cams[key].update(frame=msg, stamp=time.time())

    def _column_free(self, bgr):
        """-> per-column drivable fraction (96,), left image edge first."""
        resized = cv2.resize(bgr, (self.mw, self.mh),
                             interpolation=cv2.INTER_LINEAR)
        out = self.model.forward(bgr2nv12(resized))
        classes = np.argmax(
            out[0].buffer.reshape(19, self.mh // 8, self.mw // 8), axis=0)
        roi = classes[int(classes.shape[0] * ROI_TOP_FRAC):]
        return np.isin(roi, DRIVABLE).mean(axis=0)

    def _tick(self):
        acc = np.zeros(self.n)
        cnt = np.zeros(self.n, dtype=int)
        now = time.time()

        for c in self.cams.values():
            if c["frame"] is None or now - c["stamp"] > self.stale_s:
                continue                     # stale cam -> its sectors stay UNKNOWN
            bgr = decode(c["frame"])
            if bgr is None:
                continue
            col_free = self._column_free(bgr)
            ncols = col_free.shape[0]
            # image column -> world bearing: left edge = axis + hfov/2 (REP-103)
            bearings = c["axis"] + c["hfov"] * (0.5 - (np.arange(ncols) + 0.5) / ncols)
            idx = ((bearings - self.a0) / (self.a1 - self.a0) * self.n).astype(int)
            ok = (idx >= 0) & (idx < self.n)
            np.add.at(acc, idx[ok], col_free[ok])
            np.add.at(cnt, idx[ok], 1)

        msg = Sectors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = float(self.a0)
        msg.angle_max = float(self.a1)
        free = np.where(cnt > 0, acc / np.maximum(cnt, 1), 0.0)
        status = np.where(cnt == 0, Sectors.UNKNOWN,
                          np.where(free >= self.free_thresh,
                                   Sectors.FREE, Sectors.BLOCKED))
        msg.free_fraction = [float(f) for f in free]
        msg.status = [int(s) for s in status]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
