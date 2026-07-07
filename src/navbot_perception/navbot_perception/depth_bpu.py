#!/usr/bin/env python3
"""
depth_bpu — Depth Anything V2 (ViT-S) monocular depth on the BPU ->
/perception/grid_overlay (front camera only).

Idle (no model loaded, no BPU load) until /perception/depth_enable latches
true — lazy-loaded on first enable, same as detection_bpu. Depth Anything
outputs RELATIVE inverse depth (bright = close, dark = far); this node
min-max normalizes per frame (no metric calibration — visualization only;
see docs/depth_anything_conversion.md for a future metric-scale path via
TF-Luna) and block-reduces to the same coarse grid shape obstacle_fusion's
pidnet overlay uses, so the desktop can share one renderer.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from hobot_dnn import pyeasy_dnn as dnn
from navbot_msgs.msg import GridOverlay

from .imgproc import bgr2nv12, decode

# ViT-S 518: the conversion doc's own "start here" recommendation for a
# ~10-TOPS edge BPU (docs/depth_anything_conversion.md).
MODEL_BIN = "/home/sunrise/Desktop/RDK/model_output_vits/depth_anything_v2_vits_518.bin"
STALE_S = 0.6
GRID_ROWS = 12
GRID_COLS = 16


class DepthBpu(Node):
    def __init__(self):
        super().__init__("depth_bpu")

        self.declare_parameter("front_topic", "/cam_front/image_raw")
        self.declare_parameter("rate_hz", 6.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.enabled = False
        self.model = None                # lazy-loaded on first enable
        self.size = None
        self.frame = None
        self.stamp = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/perception/depth_enable",
                                 self._on_enable, latched)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, g("front_topic"),
                                 self._store, qos)

        self.pub = self.create_publisher(GridOverlay, "/perception/grid_overlay", 10)
        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info("depth_bpu up (idle until enabled, front camera only)")

    def _store(self, msg):
        self.frame = msg
        self.stamp = time.time()

    def _on_enable(self, msg):
        want = bool(msg.data)
        if want and self.model is None:
            self.get_logger().info("loading DepthAnything ...")
            try:
                self.model = dnn.load(MODEL_BIN)[0]
            except Exception as exc:
                # dnn.load raises (SystemError) when the BPU's ION pool can't
                # fit the model — e.g. ~225 MB for ViT-S 518 vs a 320 MB pool
                # already holding PIDNet + camera pipelines. Stay alive and
                # disabled instead of taking the whole node down.
                self.get_logger().error(
                    f"DepthAnything load failed ({exc!r}) — staying disabled;"
                    " likely out of BPU/ION memory: disable other models or"
                    " enlarge the ION pool")
                self.enabled = False
                return
            self.size = self.model.inputs[0].properties.shape[2]   # square input
            self.get_logger().info(f"DepthAnything loaded ({self.size}x{self.size})")
        elif not want and self.model is not None:
            self.model = None                      # release BPU/ION memory
            self.get_logger().info("DepthAnything unloaded")
        self.enabled = want

    def _grid_from_depth(self, depth):
        """-> (GRID_ROWS, GRID_COLS) uint8, 0-255 normalized, brighter=closer."""
        d = (depth - depth.min()) / (depth.max() - depth.min() + 1e-8)
        grid = np.empty((GRID_ROWS, GRID_COLS), dtype=np.uint8)
        for r, row_cells in enumerate(np.array_split(d, GRID_ROWS, axis=0)):
            for c, cell in enumerate(np.array_split(row_cells, GRID_COLS, axis=1)):
                grid[r, c] = int(cell.mean() * 255)
        return grid

    def _tick(self):
        if not self.enabled or self.model is None:
            return
        if self.frame is None or time.time() - self.stamp > STALE_S:
            return
        bgr = decode(self.frame)
        if bgr is None:
            return
        resized = cv2.resize(bgr, (self.size, self.size),
                             interpolation=cv2.INTER_LINEAR)
        out = self.model.forward(bgr2nv12(resized))
        depth = out[0].buffer.reshape(self.size, self.size)
        grid = self._grid_from_depth(depth)

        msg = GridOverlay()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.camera = "front"
        msg.kind = GridOverlay.KIND_DEPTH
        msg.rows = GRID_ROWS
        msg.cols = GRID_COLS
        msg.cells = [int(v) for v in grid.flatten()]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthBpu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
