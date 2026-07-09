#!/usr/bin/env python3
"""
safety_gate — the only writer of /cmd_vel_safe.

Subscribes : /cmd_vel (geometry_msgs/Twist, from local_planner or teleop)
             /obstacles (navbot_msgs/Sectors, from obstacle_fusion)
Publishes  : /cmd_vel_safe (geometry_msgs/Twist) — consumed by motor_controller
             /estop_state (std_msgs/Bool, latched) — E-stop readback for operator UIs
             /range_forward (sensor_msgs/Range) — TF-Luna range, NaN when unreadable
             /forward_blocked (std_msgs/Bool, latched) — true while forward is clamped
Service    : /estop (std_srvs/SetBool)  data=true engages the E-stop

Three independent protections on top of whatever the planner commands:
  * TF-Luna forward range (I2C5 @ 0x10, polled in-process): closer than
    `stop_cm` -> forward velocity is clamped to 0 (reverse/rotate still pass,
    so the robot can extract itself). Sensor unreadable -> fail-safe: forward
    is blocked.
  * Visual path check: every /obstacles sector within ±`front_halfwidth_deg`
    of straight ahead must be FREE, and the message must be fresher than
    `obstacles_stale_s` — otherwise forward is clamped to 0 (UNKNOWN sectors
    count as blocked, per the Sectors contract). This makes manual teleop
    respect the PIDNet free-space estimate, not just the operator's stick.
    Defaults on (the `visual_stop` param); the operator console can flip it
    live via a latched Bool on /perception/obstacle_avoidance_enable. The
    TF-Luna check above is NOT gated by this — it's a physical backstop
    that always stays on.
  * E-stop: while engaged every command is zeroed.

motor_controller's own dead-man (cmd_timeout) remains the last line: if this
node dies, motors coast to a stop.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool

from navbot_msgs.msg import Sectors

from smbus2 import SMBus, i2c_msg

TFLUNA_ADDR = 0x10


class SafetyGate(Node):
    def __init__(self):
        super().__init__("safety_gate")

        self.declare_parameter("stop_cm", 30.0)
        self.declare_parameter("i2c_bus", 5)
        self.declare_parameter("min_amp", 100)     # TF-Luna signal strength floor
        self.declare_parameter("lidar_rate_hz", 20.0)
        self.declare_parameter("visual_stop", True)
        self.declare_parameter("front_halfwidth_deg", 25.0)
        self.declare_parameter("obstacles_stale_s", 1.0)
        self.declare_parameter("visual_release_s", 0.5)   # cone must stay clear this long

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stop_cm = g("stop_cm")
        self.min_amp = g("min_amp")
        self.visual_stop = bool(g("visual_stop"))
        self.front_hw = math.radians(float(g("front_halfwidth_deg")))
        self.obstacles_stale_s = float(g("obstacles_stale_s"))
        self.visual_release_s = float(g("visual_release_s"))
        self.visual_clear_since = None             # monotonic ts cone went clear

        self.bus = SMBus(g("i2c_bus"))
        self.estop = False
        self.lidar_cm = None                       # None = unreadable
        self.sectors = None                        # last /obstacles, None = never seen
        self.sectors_t = 0.0
        self.fwd_blocked = None                    # last published /forward_blocked

        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = self.create_publisher(Bool, "/estop_state", latched)
        self.pub_estop.publish(Bool(data=False))
        self.pub_range = self.create_publisher(Range, "/range_forward", 10)
        self.pub_blocked = self.create_publisher(Bool, "/forward_blocked", latched)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Sectors, "/obstacles", self._on_obstacles, 10)
        self.create_subscription(Bool, "/perception/obstacle_avoidance_enable",
                                 self._on_visual_stop_enable, latched)
        self.create_service(SetBool, "/estop", self._on_estop)
        self.create_timer(1.0 / g("lidar_rate_hz"), self._read_lidar)
        self.get_logger().info(
            f"safety_gate up: stop at {self.stop_cm:.0f} cm, visual stop "
            f"{'on ±%.0f°' % math.degrees(self.front_hw) if self.visual_stop else 'OFF'},"
            " /estop ready")

    def _read_lidar(self):
        try:
            w = i2c_msg.write(TFLUNA_ADDR, [0x00])
            r = i2c_msg.read(TFLUNA_ADDR, 4)
            self.bus.i2c_rdwr(w, r)
            d = list(r)
            dist, amp = d[0] | d[1] << 8, d[2] | d[3] << 8
            self.lidar_cm = float(dist) if amp >= self.min_amp and dist > 0 else None
        except OSError:
            self.lidar_cm = None
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = "tf_luna"
        r.radiation_type = Range.INFRARED
        r.field_of_view = 0.035
        r.min_range, r.max_range = 0.10, 8.0
        r.range = (self.lidar_cm / 100.0) if self.lidar_cm is not None else math.nan
        self.pub_range.publish(r)
        blocked = self._lidar_unsafe() or self._visual_unsafe()
        if blocked != self.fwd_blocked:            # latched, publish on change
            self.fwd_blocked = blocked
            self.pub_blocked.publish(Bool(data=blocked))

    def _on_estop(self, req, resp):
        self.estop = req.data
        self.pub_estop.publish(Bool(data=self.estop))
        self.get_logger().warn(f"E-STOP {'ENGAGED' if self.estop else 'released'}")
        if self.estop:
            self.pub.publish(Twist())              # zero immediately
        resp.success = True
        resp.message = "engaged" if self.estop else "released"
        return resp

    def _on_obstacles(self, msg):
        self.sectors = msg
        self.sectors_t = time.monotonic()

    def _on_visual_stop_enable(self, msg):
        self.visual_stop = bool(msg.data)
        self.get_logger().info(
            "obstacle avoidance (visual stop) "
            + ("enabled" if self.visual_stop else "DISABLED") + " by operator")

    def _lidar_unsafe(self):
        return self.lidar_cm is None or self.lidar_cm < self.stop_cm

    def _cone_unclear(self):
        """Instantaneous check: True unless fresh /obstacles data marks every
        sector whose CENTER bearing is within ±front_halfwidth FREE. No data
        yet / stale data / UNKNOWN sectors all fail-safe to blocked."""
        m = self.sectors
        if m is None or time.monotonic() - self.sectors_t > self.obstacles_stale_s:
            return True
        n = len(m.status)
        if n == 0 or m.angle_max <= m.angle_min:
            return True
        w = (m.angle_max - m.angle_min) / n
        lo = max(math.ceil((-self.front_hw - m.angle_min) / w - 0.5), 0)
        hi = min(math.floor((self.front_hw - m.angle_min) / w - 0.5), n - 1)
        return any(s != Sectors.FREE for s in m.status[lo:hi + 1])

    def _visual_unsafe(self):
        """_cone_unclear with asymmetric hysteresis: blocks on the first bad
        frame, releases only after visual_release_s of continuously clear
        cone — a single flickering sector can't chatter the gate."""
        if not self.visual_stop:
            return False
        now = time.monotonic()
        if self._cone_unclear():
            self.visual_clear_since = None
            return True
        if self.visual_clear_since is None:
            self.visual_clear_since = now
        return now - self.visual_clear_since < self.visual_release_s

    def _on_cmd(self, msg):
        out = Twist()
        if not self.estop:
            out.angular = msg.angular
            out.linear = msg.linear
            if msg.linear.x > 0.0 and (self._lidar_unsafe()
                                       or self._visual_unsafe()):
                out.linear.x = 0.0                 # block forward, keep rotation
                self.get_logger().warn(
                    "forward blocked: "
                    + ("lidar" if self._lidar_unsafe() else "visual")
                    + " says path not clear",
                    throttle_duration_sec=2.0)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
