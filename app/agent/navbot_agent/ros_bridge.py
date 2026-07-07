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
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, Imu, Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from navbot_msgs.msg import Sectors

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
        self._counters = {t: 0 for t in config.RATE_TOPICS}
        self._rates_taken = time.monotonic()
        self._rates_last = {t: 0 for t in config.RATE_TOPICS}

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
        self.create_subscription(Imu, "/imu/data",
                                 lambda m: self._count("/imu/data"), sensor)
        self.create_subscription(Range, "/range_forward", self._on_range, 10)
        self.create_subscription(Bool, "/estop_state", self._on_estop_state, latched)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._estop_cli = self.create_client(SetBool, "/estop")

        self.create_timer(1.0 / config.TELEOP_RATE_HZ, self._teleop_tick)
        self.create_timer(0.1, self._estop_reconcile)
        self.get_logger().info("navbot_agent bridge node up")

    # ---------------- subscriptions ----------------

    def _count(self, topic):
        self.b._counters[topic] += 1

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
