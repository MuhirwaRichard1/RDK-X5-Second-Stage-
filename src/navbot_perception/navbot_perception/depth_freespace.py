#!/usr/bin/env python3
"""
depth_freespace — PRIMARY obstacle sensor: monocular metric-free depth ->
front free-space sectors on /obstacles.

Rationale: PIDNet (Cityscapes semantic segmentation) reads indoor floors
poorly — a street model guessing "road vs not" indoors gives unreliable
clear-path calls. Depth Anything V2 measures scene GEOMETRY instead, which is
what obstacle avoidance actually needs. This node replaces obstacle_fusion as
the /obstacles publisher in manual mode.

Model: Depth Anything V2 ViT-S at 392x392 (model_output_vits392). Chosen
empirically — it is the only converted variant that both fits the stock
320 MB ION pool AND runs fast (~2.8 Hz forward), so no bootloader/ION-resize
surgery is needed. It outputs INVERSE relative depth (larger = closer).

Free-space method (validated on captured frames, see scratchpad/freespace2.py):
for a low forward camera, clear floor recedes smoothly — scanning a column
from the robot's feet upward, inverse depth falls monotonically. A vertical
obstacle breaks that: it sits closer than the floor would at that image row,
so inverse depth stops falling. The inverse depth AT the break is the
obstacle's nearness (a physical distance proxy, robust to camera tilt); a
sector whose nearest obstacle exceeds `near_block` is BLOCKED. Columns clear
to the horizon read far (free). Also republishes the depth HUD grid so the
console depth overlay keeps working.

Path-ahead band (`path_bottom_frac`): the front camera sits very low, so the
bottom of the frame is a near-field FLOOR APRON the robot is effectively
already standing on, not steering-relevant space — and at night it is where
floor glare/reflections manufacture false "near" breaks. That zone is the
TF-Luna's jurisdiction (safety_gate clamps forward below stop_cm as an always-
on physical backstop, and the beam sits beneath this camera). So we still walk
the floor trend THROUGH the apron (real floor = a rock-solid reference for the
cumulative-min), but only DECLARE an obstacle when the break lands in the upper
path-ahead band [horizon_frac .. path_bottom_frac]. Raise path_bottom_frac to
watch more of the floor; lower it to trust the TF-Luna for more of the near
field. Constraint: horizon_frac < path_bottom_frac < feet_frac.
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
from navbot_msgs.msg import Sectors, GridOverlay

from .imgproc import bgr2nv12, decode

MODEL_BIN = "/home/sunrise/Desktop/RDK/model_output_vits392/depth_anything_v2_vits392.bin"
GRID_ROWS = 12
GRID_COLS = 16


class DepthFreespace(Node):
    def __init__(self):
        super().__init__("depth_freespace")

        self.declare_parameter("front_topic", "/cam_front/image_raw")
        self.declare_parameter("rate_hz", 4.0)
        self.declare_parameter("n_sectors", 12)
        self.declare_parameter("front_hfov_deg", 70.0)
        self.declare_parameter("front_axis_deg", 0.0)
        self.declare_parameter("stale_s", 0.8)
        # free-space tuning (see module docstring / scratchpad validation)
        self.declare_parameter("floor_tol", 0.15)    # inv-depth rise = obstacle
        self.declare_parameter("near_block", 2.2)     # inv-depth >= this => BLOCKED
        self.declare_parameter("horizon_frac", 0.30)  # ignore top (walls/ceiling)
        self.declare_parameter("path_bottom_frac", 0.55)  # ignore below (floor apron/TF-Luna zone)
        self.declare_parameter("feet_frac", 0.90)     # bottom = floor-at-feet seed
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.n = int(g("n_sectors"))
        self.hfov = np.radians(float(g("front_hfov_deg")))
        self.axis = np.radians(float(g("front_axis_deg")))
        self.a0 = self.axis - self.hfov / 2.0
        self.a1 = self.axis + self.hfov / 2.0
        self.stale_s = float(g("stale_s"))
        self.floor_tol = float(g("floor_tol"))
        self.near_block = float(g("near_block"))
        self.horizon_frac = float(g("horizon_frac"))
        self.path_bottom_frac = float(g("path_bottom_frac"))
        self.feet_frac = float(g("feet_frac"))

        self.frame = None
        self.stamp = 0.0
        self.overlay_enabled = False

        # model is the safety sensor -> load on startup, not lazily. If it
        # can't fit ION we keep spinning and publish all-UNKNOWN (fail-safe:
        # the safety gate treats UNKNOWN as blocked), rather than dying.
        self.model = None
        self.size = None
        try:
            self.model = dnn.load(MODEL_BIN)[0]
            self.size = self.model.inputs[0].properties.shape[2]
            self.get_logger().info(f"DepthAnything vits392 loaded ({self.size}x{self.size})")
        except Exception as exc:
            self.get_logger().error(
                f"depth model load failed ({exc!r}) — publishing UNKNOWN "
                "sectors (fail-safe blocks forward). Free ION or check the model.")

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/perception/depth_enable",
                                 self._on_overlay_enable, latched)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, g("front_topic"),
                                 self._store, qos)

        self.pub = self.create_publisher(Sectors, "/obstacles", 10)
        self.grid_pub = self.create_publisher(GridOverlay,
                                              "/perception/grid_overlay", 10)
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        self.get_logger().info(
            f"depth_freespace up: {self.n} sectors over "
            f"[{np.degrees(self.a0):.0f}..{np.degrees(self.a1):.0f}]deg, "
            f"near_block={self.near_block}, path band "
            f"[{self.horizon_frac:.2f}..{self.path_bottom_frac:.2f}] "
            f"(apron below {self.path_bottom_frac:.2f} -> TF-Luna)")

    def _store(self, msg):
        self.frame = msg
        self.stamp = time.time()

    def _on_overlay_enable(self, msg):
        self.overlay_enabled = bool(msg.data)

    # ---- free-space core (per-column floor-break nearness, vectorized) ----
    def _column_nearness(self, depth):
        """Per column: inverse-depth of the nearest floor-breaking obstacle
        (0 if clear floor to the horizon). Walking each column up from the
        feet, an obstacle is the first row whose inverse depth exceeds the
        running floor minimum (below it) by > floor_tol. Vectorized across
        columns via a cumulative-min; bit-identical to the reference scan."""
        h, w = depth.shape
        horizon = int(h * self.horizon_frac)
        feet = int(h * self.feet_frac)
        seed = depth[feet:].min(axis=0)                     # (w,) floor at feet
        rows_up = depth[horizon + 1:feet][::-1]             # k=0 -> feet-1, upward
        stacked = np.concatenate([seed[None, :], rows_up], axis=0)
        running_before = np.minimum.accumulate(stacked, axis=0)[:-1]  # min below row
        break_mask = rows_up > running_before + self.floor_tol
        # Only DECLARE obstacles in the upper path-ahead band: silence breaks in
        # the near-field floor apron (rows below path_bottom), which the TF-Luna
        # backstop owns and where night floor-glare fabricates false breaks. The
        # floor trend above was still tracked through the apron. rows_up[k] is
        # image row feet-1-k, so apron rows are the first k_min entries.
        path_bottom = int(h * self.path_bottom_frac)
        k_min = min(max(0, feet - 1 - path_bottom), break_mask.shape[0])
        if k_min:
            break_mask[:k_min] = False
        near = np.zeros(w, dtype=np.float32)
        any_break = break_mask.any(axis=0)
        first_k = break_mask.argmax(axis=0)                 # first break per column
        cols = np.where(any_break)[0]
        near[cols] = rows_up[first_k[cols], cols]
        return near

    def _sectors_from_depth(self, depth):
        """-> (status list, free_fraction list) over the front FOV."""
        near = self._column_nearness(depth)
        w = len(near)
        per = max(w // self.n, 1)
        status, free = [], []
        for s in range(self.n):
            seg = near[s * per:(s + 1) * per] if s < self.n - 1 else near[s * per:]
            v = float(np.percentile(seg, 90)) if seg.size else 0.0
            status.append(Sectors.BLOCKED if v >= self.near_block else Sectors.FREE)
            free.append(float(max(0.0, 1.0 - v / self.near_block)))
        return status, free

    def _depth_grid(self, depth):
        d = (depth - depth.min()) / (depth.max() - depth.min() + 1e-8)
        grid = np.empty((GRID_ROWS, GRID_COLS), dtype=np.uint8)
        for r, band in enumerate(np.array_split(d, GRID_ROWS, axis=0)):
            for c, cell in enumerate(np.array_split(band, GRID_COLS, axis=1)):
                grid[r, c] = int(cell.mean() * 255)
        return grid

    def _publish_unknown(self):
        msg = Sectors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min, msg.angle_max = float(self.a0), float(self.a1)
        msg.status = [Sectors.UNKNOWN] * self.n
        msg.free_fraction = [0.0] * self.n
        self.pub.publish(msg)

    def _tick(self):
        # No model or no fresh frame -> fail-safe UNKNOWN (gate blocks forward).
        if self.model is None:
            self._publish_unknown()
            return
        if self.frame is None or time.time() - self.stamp > self.stale_s:
            self._publish_unknown()
            return
        bgr = decode(self.frame)
        if bgr is None:
            self._publish_unknown()
            return
        resized = cv2.resize(bgr, (self.size, self.size),
                             interpolation=cv2.INTER_LINEAR)
        depth = self.model.forward(bgr2nv12(resized))[0].buffer.reshape(
            self.size, self.size).astype(np.float32)
        depth = cv2.medianBlur(depth, 5)      # denoise floor reflections

        status, free = self._sectors_from_depth(depth)
        msg = Sectors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min, msg.angle_max = float(self.a0), float(self.a1)
        msg.status = [int(s) for s in status]
        msg.free_fraction = [float(f) for f in free]
        self.pub.publish(msg)

        if self.overlay_enabled:
            grid = self._depth_grid(depth)
            gmsg = GridOverlay()
            gmsg.header.stamp = msg.header.stamp
            gmsg.header.frame_id = "base_link"
            gmsg.camera = "front"
            gmsg.kind = GridOverlay.KIND_DEPTH
            gmsg.rows, gmsg.cols = GRID_ROWS, GRID_COLS
            gmsg.cells = [int(v) for v in grid.flatten()]
            self.grid_pub.publish(gmsg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFreespace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
