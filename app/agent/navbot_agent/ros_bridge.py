"""RosBridge — the agent's single rclpy node, spun in a daemon thread.

Thread contract:
  * All rclpy calls happen on the executor thread (callbacks + timers).
  * asyncio -> ROS: plain attribute writes of immutable tuples/bools
    (atomic under the GIL); the node's timers read them.
  * ROS -> asyncio: image frames land in latest-wins slots read by the
    video pump; low-rate events go through loop.call_soon_threadsafe.

Safety:
  * Teleop is published to /cmd_vel at 20 Hz ONLY while teleop_enabled
    (manual mode), the last command is < 0.4 s old, and the operator
    E-stop latch is off. Going stale sends exactly one zero Twist.
  * The E-stop reconciler keeps safety_gate's latch equal to the
    operator's intent — including after a mode switch restarts
    safety_gate (its internal latch resets; /estop_state readback lets
    us detect and re-assert)."""

import math
import os
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage, Image, Imu, Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from navbot_msgs.msg import Detections, GridOverlay, Sectors

from . import config, protocol


class RosBridge:
    def __init__(self):
        self.app = None
        self.loop = None
        self.node = None
        self._executor = None
        self._thread = None

        # ---- shared state (atomic writes, single writer each) ----
        # cam -> ("jpeg", bytes, seq, mono_ms) | ("yuyv", bytes, w, h, seq, mono_ms)
        self.frame_slots = {cam: None for cam in config.CAMERAS}
        self._range = None              # (cm-or-NaN, t_mono); stale > 1.5 s -> None
        self.estop_confirmed = None     # /estop_state readback (None = unknown)
        self.estop_intent = None        # operator latch (None = untouched)
        self.teleop_enabled = False     # set by launch manager (manual mode)
        self._teleop = None             # (vx, wz, t_mono)
        self.model_intent = dict(config.MODEL_DEFAULTS)  # operator toggle
        self._counters = {t: 0 for t in config.RATE_TOPICS}
        self._rates_taken = time.monotonic()
        self._rates_last = {t: 0 for t in config.RATE_TOPICS}
        self.attitude = None            # (roll°, pitch°, yaw°, yaw_rate°/s, t_mono)
        self.map_slot = None            # (data, width, height, res, ox, oy, seq, mono_ms)
        self.robot_pose = None          # (x, y, yaw_rad, t_mono) from map->base_link TF
        self.pending_save = None        # map basename to save (set on asyncio thread)
        self._goal = None               # (x, y, t_mono) newest nav goal, map frame

    # ---------------- lifecycle ----------------

    def start(self, app, loop):
        self.app = app
        self.loop = loop
        rclpy.init()
        self.node = _AgentNode(self)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin,
                                        name="rclpy-executor", daemon=True)
        self._thread.start()

    def stop(self):
        if self._executor:
            self._executor.shutdown(timeout_sec=2.0)
        if self.node:
            self.node.destroy_node()
        rclpy.try_shutdown()
        if self._thread:
            self._thread.join(timeout=3.0)

    # ---------------- asyncio-side API ----------------

    def set_teleop(self, vx, wz):
        self._teleop = (vx, wz, time.monotonic())

    def teleop_age_ms(self):
        t = self._teleop
        return int((time.monotonic() - t[2]) * 1000) if t else None

    def request_estop(self, engage):
        self.estop_intent = bool(engage)

    def set_model_enable(self, model, enable):
        self.model_intent = {**self.model_intent, model: bool(enable)}

    def save_map(self, base):
        """Request a map save (picked up by the executor thread's timer)."""
        self.pending_save = base

    def set_goal(self, x, y):
        """Set the newest navigation goal (map frame); published by the node."""
        self._goal = (float(x), float(y), time.monotonic())

    def reassert_models(self):
        """Force every model-enable topic to be republished on the next
        reconcile tick — call after a mode switch restarts the perception
        nodes, since their enable flags reset to False but our publish
        cache doesn't know that happened."""
        if self.node:
            self.node.force_model_reassert()

    @property
    def range_cm(self):
        r = self._range
        if r is None or time.monotonic() - r[1] > 1.5:
            return None
        return r[0]

    def take_rates(self):
        now = time.monotonic()
        dt = max(now - self._rates_taken, 1e-3)
        self._rates_taken = now
        rates = {}
        for topic, n in self._counters.items():
            rates[topic] = round((n - self._rates_last[topic]) / dt, 1)
            self._rates_last[topic] = n
        return rates

    # ---------------- executor-thread helpers ----------------

    def _post(self, fn, *args):
        """Run fn(*args) on the asyncio loop thread."""
        if self.loop and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(fn, *args)


class _AgentNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("navbot_agent")
        self.b = bridge
        self._seq = {cam: 0 for cam in config.CAMERAS}
        self._teleop_active = False     # we owe a zero Twist when going stale
        self._estop_inflight_t = None

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(CompressedImage, "/cam_front/image_raw",
                                 self._on_front, sensor)
        self.create_subscription(Image, "/cam_left/image_raw",
                                 lambda m: self._on_side("left", m), sensor)
        self.create_subscription(Image, "/cam_right/image_raw",
                                 lambda m: self._on_side("right", m), sensor)
        self.create_subscription(Sectors, "/obstacles", self._on_obstacles, sensor)
        self.create_subscription(Twist, "/cmd_vel",
                                 lambda m: self._count("/cmd_vel"), 10)
        self.create_subscription(Twist, "/cmd_vel_safe",
                                 lambda m: self._count("/cmd_vel_safe"), 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor)
        # complementary-filter state (executor thread only)
        self._att = [0.0, 0.0, 0.0]     # roll, pitch, yaw (rad)
        self._att_t = None              # last msg stamp (s)
        self._att_ready = False         # first sample snaps to accel angles
        self.create_subscription(Range, "/range_forward", self._on_range, 10)
        self.create_subscription(Bool, "/estop_state", self._on_estop_state, latched)

        self.create_subscription(GridOverlay, "/perception/grid_overlay",
                                 self._on_grid_overlay, sensor)
        self.create_subscription(Detections, "/perception/detections",
                                 self._on_detections, sensor)
        self._model_pub = {m: self.create_publisher(Bool, topic, latched)
                           for m, topic in config.MODEL_ENABLE_TOPIC.items()}
        self._model_published = {m: False for m in config.MODELS}

        self._map_seq = 0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._estop_cli = self.create_client(SetBool, "/estop")

        # SLAM map save (both viewable pgm/yaml and loadable posegraph/data)
        # and navigation goal publishing.
        self._save_map_cli = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self._serialize_cli = self.create_client(SerializePoseGraph,
                                                 "/slam_toolbox/serialize_map")
        self._save_wait = 0
        self._goal_pub = self.create_publisher(PoseStamped, "/goal", latched)
        self._goal_published_t = None

        self.create_timer(1.0 / config.TELEOP_RATE_HZ, self._teleop_tick)
        self.create_timer(0.1, self._estop_reconcile)
        self.create_timer(0.5, self._model_reconcile)
        self.create_timer(0.5, self._pose_tick)
        self.create_timer(0.3, self._save_map_tick)
        self.create_timer(0.2, self._goal_tick)
        self.get_logger().info("navbot_agent bridge node up")

    # ---------------- subscriptions ----------------

    def _count(self, topic):
        self.b._counters[topic] += 1

    def _on_imu(self, m):
        """200 Hz complementary filter over the raw MPU6050 stream.

        imu_driver publishes REP-103 body axes (x fwd, y left, z up), rates
        bias-corrected, accel auto-scaled to |g|. bridge.attitude is stored
        in GCS display convention: roll + = bank right, pitch + = nose up,
        yaw/heading clockwise 0-360° relative to power-on (gyro-only, drifts)."""
        self._count("/imu/data")
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ax, ay, az = (m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        acc_ok = 4.9 < norm < 14.7          # ~0.5..1.5 g: usable for leveling
        dt = 0.0 if self._att_t is None else min(max(t - self._att_t, 0.0), 0.05)
        self._att_t = t

        if not self._att_ready:
            if not acc_ok:
                return
            self._att = [math.atan2(ay, az),
                         math.atan2(-ax, math.hypot(ay, az)), 0.0]
            self._att_ready = True
        else:
            r, p, y = self._att
            r += m.angular_velocity.x * dt   # small-angle Euler: fine for a
            p += m.angular_velocity.y * dt   # ground robot's roll/pitch range
            y += m.angular_velocity.z * dt
            if acc_ok:                       # 0.98 @ 200 Hz -> tau ~ 0.25 s
                r = 0.98 * r + 0.02 * math.atan2(ay, az)
                p = 0.98 * p + 0.02 * math.atan2(-ax, math.hypot(ay, az))
            y = (y + math.pi) % (2 * math.pi) - math.pi
            self._att = [r, p, y]

        r, p, y = self._att
        self.b.attitude = (
            math.degrees(r) - config.ATT_TRIM_ROLL_DEG,
            -math.degrees(p) - config.ATT_TRIM_PITCH_DEG,
            (-math.degrees(y)) % 360.0,
            -math.degrees(m.angular_velocity.z),
            time.monotonic())

    def _on_front(self, msg):
        self._count("/cam_front/image_raw")
        self._seq["front"] += 1
        self.b.frame_slots["front"] = (
            "jpeg", bytes(msg.data), self._seq["front"],
            int(time.monotonic() * 1000))

    def _on_side(self, cam, msg):
        self._count(f"/cam_{cam}/image_raw")
        self._seq[cam] += 1
        self.b.frame_slots[cam] = (
            "yuyv", bytes(msg.data), msg.width, msg.height, self._seq[cam],
            int(time.monotonic() * 1000))

    def _on_obstacles(self, msg):
        self._count("/obstacles")
        b = self.b
        if b.app and b.app.hub and b.app.hub.sessions:
            b._post(b.app.hub.broadcast_fast, protocol.sectors(
                round(msg.angle_min, 4), round(msg.angle_max, 4),
                list(msg.status), [round(f, 3) for f in msg.free_fraction]))

    def _on_grid_overlay(self, msg):
        b = self.b
        if b.app and b.app.hub and b.app.hub.sessions:
            b._post(b.app.hub.broadcast_fast, protocol.grid_overlay(
                msg.camera, int(msg.kind), int(msg.rows), int(msg.cols),
                list(msg.cells)))

    def _on_detections(self, msg):
        b = self.b
        if b.app and b.app.hub and b.app.hub.sessions:
            boxes = [{"x1": round(x1, 4), "y1": round(y1, 4),
                      "x2": round(x2, 4), "y2": round(y2, 4),
                      "score": round(score, 3), "class_name": name}
                     for x1, y1, x2, y2, score, name in
                     zip(msg.x1, msg.y1, msg.x2, msg.y2, msg.score, msg.class_name)]
            b._post(b.app.hub.broadcast_fast, protocol.detections(msg.camera, boxes))

    def _on_map(self, msg):
        self._map_seq += 1
        info = msg.info
        # msg.data is signed int8 (-1 unknown, 0..100 occupancy probability);
        # bytes() rejects negatives, so go through numpy to preserve the
        # signed byte pattern instead of the (positive) Python int values.
        data = np.array(msg.data, dtype=np.int8).tobytes()
        self.b.map_slot = (
            data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            self._map_seq, int(time.monotonic() * 1000))

    def _on_range(self, msg):
        self._count("/range_forward")
        cm = msg.range * 100.0 if not math.isnan(msg.range) else math.nan
        self.b._range = (cm, time.monotonic())

    def _on_estop_state(self, msg):
        changed = msg.data != self.b.estop_confirmed
        self.b.estop_confirmed = msg.data
        if changed and self.b.app:
            self.b._post(self.b.app.broadcast_state)

    # ---------------- timers ----------------

    def _teleop_tick(self):
        b = self.b
        t = b._teleop
        fresh = (t is not None
                 and time.monotonic() - t[2] < config.TELEOP_STALE_S)
        if b.teleop_enabled and fresh and not b.estop_intent:
            out = Twist()
            out.linear.x, out.angular.z = t[0], t[1]
            self._cmd_pub.publish(out)
            self._teleop_active = True
        elif self._teleop_active:
            self._cmd_pub.publish(Twist())      # exactly one zero, then silence
            self._teleop_active = False

    def _estop_reconcile(self):
        """Keep safety_gate's latch equal to operator intent — re-asserts
        automatically after a mode switch restarts safety_gate."""
        b = self.b
        if b.estop_intent is None or b.estop_intent == b.estop_confirmed:
            return
        now = time.monotonic()
        if self._estop_inflight_t and now - self._estop_inflight_t < 2.0:
            return
        if not self._estop_cli.service_is_ready():
            return
        self._estop_inflight_t = now
        fut = self._estop_cli.call_async(SetBool.Request(data=b.estop_intent))
        fut.add_done_callback(self._estop_done)

    def force_model_reassert(self):
        """None never equals a bool, so the next reconcile tick republishes
        every model-enable topic regardless of its last-published value."""
        self._model_published = {m: None for m in config.MODELS}

    def _model_reconcile(self):
        """Publish a latched Bool for any model toggle that changed —
        moves the operator's intent (set on the asyncio thread) onto the
        ROS executor thread."""
        intent = self.b.model_intent
        for model, want in intent.items():
            if want != self._model_published[model]:
                self._model_pub[model].publish(Bool(data=want))
                self._model_published[model] = want

    def _pose_tick(self):
        """2 Hz poll of the map->base_link transform for the console's map
        marker. No-op (leaves the last known pose to go stale on its own)
        until SLAM is actually running and has processed at least one scan
        — same 'quietly not ready yet' shape as _estop_reconcile."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.b.robot_pose = (t.transform.translation.x,
                             t.transform.translation.y, yaw, time.monotonic())

    def _alog(self, level, text):
        if self.b.app:
            self.b._post(self.b.app.add_log, "agent", level, text)

    def _save_map_tick(self):
        base = self.b.pending_save
        if base is None:
            return
        if not (self._save_map_cli.service_is_ready()
                and self._serialize_cli.service_is_ready()):
            self._save_wait += 1
            if self._save_wait > 15:               # ~4.5 s of no slam_toolbox
                self.b.pending_save = None
                self._save_wait = 0
                self._alog("error", "save_map: slam_toolbox not running "
                                    "(start a mapping mode first)")
            return
        self._save_wait = 0
        self.b.pending_save = None
        try:
            os.makedirs(config.MAP_DIR, exist_ok=True)
        except OSError as e:
            self._alog("error", f"save_map: {e}")
            return
        path = os.path.join(config.MAP_DIR, base)
        req = SaveMap.Request()
        req.name.data = path                       # -> <path>.pgm + .yaml
        self._save_map_cli.call_async(req).add_done_callback(
            lambda f, p=path: self._save_done("pgm/yaml", p, f))
        sreq = SerializePoseGraph.Request()
        sreq.filename = path                       # -> <path>.posegraph + .data
        self._serialize_cli.call_async(sreq).add_done_callback(
            lambda f, p=path: self._save_done("posegraph", p, f))

    def _save_done(self, kind, path, fut):
        try:
            ok = fut.result().result == 0
        except Exception as e:                                  # noqa: BLE001
            ok, path = False, str(e)
        self._alog("info" if ok else "error",
                   f"map {kind} {'saved' if ok else 'FAILED'}: {path}")

    def _goal_tick(self):
        g = self.b._goal
        if g is None or g[2] == self._goal_published_t:
            return
        self._goal_published_t = g[2]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = g[0]
        msg.pose.position.y = g[1]
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _estop_done(self, fut):
        self._estop_inflight_t = None
        try:
            resp = fut.result()
            ok, msg = resp.success, resp.message
        except Exception as e:                                  # noqa: BLE001
            ok, msg = False, str(e)
        if self.b.app:
            self.b._post(self.b.app.add_log, "agent",
                         "info" if ok else "error", f"/estop call: {msg}")
