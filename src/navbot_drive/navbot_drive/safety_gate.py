#!/usr/bin/env python3
"""
safety_gate — the only writer of /cmd_vel_safe.

Subscribes : /cmd_vel (geometry_msgs/Twist, from local_planner or teleop)
             /scan (sensor_msgs/LaserScan, from sllidar_node — RPLidar C1)
             /obstacles (navbot_msgs/Sectors, from scan_sectors)
Publishes  : /cmd_vel_safe (geometry_msgs/Twist) — consumed by motor_controller
             /estop_state (std_msgs/Bool, latched) — E-stop readback for operator UIs
             /range_forward (sensor_msgs/Range) — nearest return in the forward
                            cone, NaN when the scan is stale/absent
             /forward_blocked (std_msgs/Bool, latched) — true while forward is clamped
Service    : /estop (std_srvs/SetBool)  data=true engages the E-stop

Protections on top of whatever the planner/operator commands:
  * Proximity ring (360°, always on): any /scan return inside the
    `stop_diameter_cm` safety circle stops motion TOWARD it — forward is
    zeroed while the intruder is in the front half-plane, reverse while it
    is behind; with intruders both sides only rotation passes, so the robot
    can always turn free and back away.
  * Scan forward stop (always on): nearest finite return within
    ±`forward_halfwidth_deg` of straight ahead closer than `stop_cm` ->
    forward velocity is clamped to 0. Scan stale/missing, or a cone with no
    finite returns (all self-hit/blind — e.g. rammed under the C1's 5 cm
    floor), fail-safe to blocked. Reverse is NOT blocked on stale data —
    extraction must survive a dead lidar.
  * Sector path check: every /obstacles sector within ±`front_halfwidth_deg`
    of straight ahead must be FREE, and the message fresher than
    `obstacles_stale_s` — otherwise forward is clamped to 0 (UNKNOWN sectors
    count as blocked, per the Sectors contract). Wider and earlier than the
    hard stop (scan_sectors blocks at 0.5 m vs stop_cm's 30 cm). Defaults on
    (the `sector_stop` param); the operator console can flip it live via a
    latched Bool on /perception/obstacle_avoidance_enable. The two stops
    above are NOT gated by this toggle.
  * E-stop: while engaged every command is zeroed.

Steering assist (`assist` param, manual mode; also gated by the operator
toggle): while forward is commanded and the nearest return within
±`assist_cone_deg` is closer than `assist_avoid_m`, forward speed is scaled
down and a bounded angular bias steers toward whichever side arc
(±[10°..`assist_side_deg`]) is clearer — the robot deviates slightly around
the obstacle instead of driving at it, and if the sector check has already
zeroed forward the bias turns it in place toward the clear side until the
path opens. The operator's own rotation command is never reduced, only
biased by at most `assist_w_max`.

motor_controller's own dead-man (cmd_timeout) remains the last line: if this
node dies, motors coast to a stop.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, Range
from std_srvs.srv import SetBool

from navbot_msgs.msg import Sectors


class SafetyGate(Node):
    def __init__(self):
        super().__init__("safety_gate")

        self.declare_parameter("stop_cm", 30.0)
        self.declare_parameter("stop_diameter_cm", 60.0)
        self.declare_parameter("forward_halfwidth_deg", 15.0)
        self.declare_parameter("min_range_m", 0.12)   # under -> self-hit, drop
        self.declare_parameter("scan_stale_s", 0.6)
        self.declare_parameter("yaw_offset_deg", 0.0)  # laser -> robot frame
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("sector_stop", True)
        self.declare_parameter("front_halfwidth_deg", 25.0)
        self.declare_parameter("obstacles_stale_s", 1.0)
        self.declare_parameter("sector_release_s", 0.5)  # cone must stay clear this long
        self.declare_parameter("assist", False)
        self.declare_parameter("assist_avoid_m", 0.8)  # nearer -> start deviating
        self.declare_parameter("assist_cone_deg", 30.0)
        self.declare_parameter("assist_side_deg", 60.0)
        self.declare_parameter("assist_gain", 0.6)     # rad/s at zero distance
        self.declare_parameter("assist_w_max", 0.5)    # bias cap
        self.declare_parameter("assist_v_floor", 0.3)  # min forward scale

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stop_cm = g("stop_cm")
        self.stop_radius = float(g("stop_diameter_cm")) / 200.0  # cm dia -> m radius
        self.fwd_hw = math.radians(float(g("forward_halfwidth_deg")))
        self.min_range = float(g("min_range_m"))
        self.scan_stale_s = float(g("scan_stale_s"))
        self.yaw_off = math.radians(float(g("yaw_offset_deg")))
        self.sector_stop = bool(g("sector_stop"))
        self.front_hw = math.radians(float(g("front_halfwidth_deg")))
        self.obstacles_stale_s = float(g("obstacles_stale_s"))
        self.sector_release_s = float(g("sector_release_s"))
        self.sector_clear_since = None             # monotonic ts cone went clear
        self.assist_sign = 1.0                     # deviation side memory, +1 = left
        self.assist = bool(g("assist"))
        self.assist_avoid = float(g("assist_avoid_m"))
        self.assist_cone = math.radians(float(g("assist_cone_deg")))
        self.assist_side = math.radians(float(g("assist_side_deg")))
        self.assist_gain = float(g("assist_gain"))
        self.assist_w_max = float(g("assist_w_max"))
        self.assist_v_floor = float(g("assist_v_floor"))

        self.estop = False
        self.fwd_m = None                          # nearest forward return, None = unsafe
        self.prox_front = None                     # nearest return, front half-plane
        self.prox_rear = None                      # nearest return, rear half-plane
        self.near_front = None                     # nearest within ±assist_cone
        self.left_min = None                       # nearest in left side arc
        self.right_min = None                      # nearest in right side arc
        self.scan_t = 0.0                          # monotonic rx time of last scan
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
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Sectors, "/obstacles", self._on_obstacles, 10)
        self.create_subscription(Bool, "/perception/obstacle_avoidance_enable",
                                 self._on_sector_stop_enable, latched)
        self.create_service(SetBool, "/estop", self._on_estop)
        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info(
            f"safety_gate up: scan stop at {self.stop_cm:.0f} cm "
            f"±{math.degrees(self.fwd_hw):.0f}°, proximity ring "
            f"⌀{2 * self.stop_radius:.2f} m, sector stop "
            f"{'on ±%.0f°' % math.degrees(self.front_hw) if self.sector_stop else 'OFF'}, "
            f"assist {'ON' if self.assist else 'off'}, /estop ready")

    def _on_scan(self, msg):
        """One pass over the scan -> the clearances every check needs."""
        fwd = prox_f = prox_r = near_f = left = right = None
        a = msg.angle_min + self.yaw_off
        for r in msg.ranges:
            b = math.atan2(math.sin(a), math.cos(a))   # normalize to [-pi, pi]
            a += msg.angle_increment
            if not math.isfinite(r) or r < self.min_range:
                continue
            ab = abs(b)
            if ab <= math.pi / 2.0:
                if prox_f is None or r < prox_f:
                    prox_f = r
            elif prox_r is None or r < prox_r:
                prox_r = r
            if ab <= self.fwd_hw and (fwd is None or r < fwd):
                fwd = r
            if ab <= self.assist_cone and (near_f is None or r < near_f):
                near_f = r
            if math.radians(10.0) <= b <= self.assist_side:
                if left is None or r < left:
                    left = r
            elif -self.assist_side <= b <= -math.radians(10.0):
                if right is None or r < right:
                    right = r
        self.fwd_m, self.prox_front, self.prox_rear = fwd, prox_f, prox_r
        self.near_front, self.left_min, self.right_min = near_f, left, right
        self.scan_t = time.monotonic()

    def _tick(self):
        stale = time.monotonic() - self.scan_t > self.scan_stale_s
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = "laser"
        r.radiation_type = Range.INFRARED
        r.field_of_view = 2.0 * self.fwd_hw
        r.min_range, r.max_range = 0.05, 16.0
        r.range = self.fwd_m if (self.fwd_m is not None and not stale) else math.nan
        self.pub_range.publish(r)
        blocked = self._scan_unsafe() or self._sector_unsafe()
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

    def _on_sector_stop_enable(self, msg):
        self.sector_stop = bool(msg.data)
        self.get_logger().info(
            "obstacle avoidance (sector stop + assist) "
            + ("enabled" if self.sector_stop else "DISABLED") + " by operator")

    def _scan_fresh(self):
        return time.monotonic() - self.scan_t <= self.scan_stale_s

    def _scan_unsafe(self):
        """Fail-safe forward stop: stale scan, no finite forward return
        (blind-close or driver dead), a return under stop_cm, or an intruder
        inside the proximity ring's front half all block."""
        if not self._scan_fresh():
            return True
        if self.prox_front is not None and self.prox_front < self.stop_radius:
            return True
        return self.fwd_m is None or self.fwd_m * 100.0 < self.stop_cm

    def _reverse_unsafe(self):
        """Reverse blocks only on a POSITIVE rear intrusion — never on stale
        data, so the robot can still back out with a dead lidar."""
        return (self._scan_fresh() and self.prox_rear is not None
                and self.prox_rear < self.stop_radius)

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

    def _sector_unsafe(self):
        """_cone_unclear with asymmetric hysteresis: blocks on the first bad
        frame, releases only after sector_release_s of continuously clear
        cone — a single flickering sector can't chatter the gate."""
        if not self.sector_stop:
            return False
        now = time.monotonic()
        if self._cone_unclear():
            self.sector_clear_since = None
            return True
        if self.sector_clear_since is None:
            self.sector_clear_since = now
        return now - self.sector_clear_since < self.sector_release_s

    def _assist_bias(self):
        """(forward scale, angular bias) deviating toward the clearer side
        arc while something sits within assist_avoid_m of the front cone."""
        if not (self.assist and self.sector_stop and self._scan_fresh()):
            return 1.0, 0.0
        if self.near_front is None or self.near_front >= self.assist_avoid:
            return 1.0, 0.0
        closeness = 1.0 - self.near_front / self.assist_avoid   # 0..1
        left = self.left_min if self.left_min is not None else math.inf
        right = self.right_min if self.right_min is not None else math.inf
        # near-tie (obstacle dead ahead, sides equally clear): keep deviating
        # toward the last chosen side instead of freezing or dithering
        if not math.isclose(left, right, rel_tol=0.05):
            self.assist_sign = 1.0 if left > right else -1.0  # +z = left
        bias = math.copysign(min(self.assist_gain * closeness,
                                 self.assist_w_max), self.assist_sign)
        scale = max(self.assist_v_floor, 1.0 - closeness)
        return scale, bias

    def _on_cmd(self, msg):
        out = Twist()
        if not self.estop:
            out.angular = msg.angular
            out.linear = msg.linear
            if msg.linear.x > 0.0:
                scale, bias = self._assist_bias()
                if self._scan_unsafe() or self._sector_unsafe():
                    out.linear.x = 0.0             # block forward, keep rotation
                    self.get_logger().warn(
                        "forward blocked: "
                        + ("scan" if self._scan_unsafe() else "sectors")
                        + " say path not clear",
                        throttle_duration_sec=2.0)
                else:
                    out.linear.x = msg.linear.x * scale
                # bias applies even while blocked: the robot turns toward
                # the clear side until the cone opens, then rolls forward
                out.angular.z = msg.angular.z + bias
            elif msg.linear.x < 0.0 and self._reverse_unsafe():
                out.linear.x = 0.0
                self.get_logger().warn(
                    "reverse blocked: obstacle inside the proximity ring",
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
