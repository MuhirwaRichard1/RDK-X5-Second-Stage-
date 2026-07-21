#!/usr/bin/env python3
"""
goal_navigator — drive to a map-frame goal, avoiding obstacles.

The autonomous counterpart to local_planner: instead of just seeking open
space, it seeks a GOAL while reusing the same widest-free-run avoidance so it
never drives forward into a BLOCKED/UNKNOWN sector. Runs in the "navigate"
mode alongside slam_toolbox localization (which supplies map->base_link).

Subscribes:
  /goal       geometry_msgs/PoseStamped  (map frame — from the console click)
  /obstacles  navbot_msgs/Sectors        (robot-frame free-space, scan_sectors)
  /amcl_pose  PoseWithCovarianceStamped  (localization quality — convergence)
  TF map->base_link                       (robot pose; amcl + icp odom)
Publishes:
  /cmd_vel          geometry_msgs/Twist   (-> safety_gate -> /cmd_vel_safe)
  /behaviour/state  std_msgs/String
                    (LOCALIZING/IDLE/NAVIGATE/AVOID/ARRIVED/LOST/RELOCALIZE)

On startup the robot does not know where it is inside the loaded map, so it
asks AMCL to scatter particles over the whole map
(/reinitialize_global_localization) and then WANDERS toward open space until
the filter converges. It has to drive, not spin: the C1 is a 360 deg lidar, so
turning on the spot returns the same ranges index-shifted and tells the filter
nothing — only visiting different positions disambiguates. Goals are ignored
until it has converged.

FSM per tick:
  not yet localized               -> LOCALIZING, wander toward open space
  no goal                         -> IDLE, stop
  recent lift / scan jump         -> RELOCALIZE, rotate slowly to re-localize
  no/old map->base_link TF        -> LOST, stop (also the kidnapped state)
  within goal_tolerance           -> ARRIVED, stop, clear goal
  goal not ahead (|bearing|>align)-> rotate in place toward it
  goal ahead & cone FREE          -> drive toward it (P heading, v scaled by dist)
  goal ahead but blocked          -> AVOID: steer to the widest FREE run
  nowhere free                    -> rotate-search (keeps last direction)

safety_gate stays underneath untouched (scan hard-stop + proximity + E-stop).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

from navbot_msgs.msg import Sectors

_G = 9.80665            # m/s^2 — stationary accel magnitude


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GoalNavigator(Node):
    def __init__(self):
        super().__init__("goal_navigator")

        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("v_max", 0.22)           # m/s forward
        self.declare_parameter("w_max", 1.0)            # rad/s
        self.declare_parameter("goal_tolerance", 0.15)  # m -> ARRIVED
        self.declare_parameter("heading_align", 0.35)   # rad; drive only if aligned
        self.declare_parameter("steer_gain", 1.5)       # w per rad bearing error
        self.declare_parameter("front_cone_deg", 25.0)  # must be FREE to drive
        self.declare_parameter("min_run_deg", 25.0)     # narrower FREE runs ignored
        self.declare_parameter("slow_dist", 0.6)        # m; scale v down inside this
        self.declare_parameter("pose_stale_s", 1.0)     # no TF this long -> LOST
        self.declare_parameter("obstacles_stale_s", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # kidnap recovery: an accel spike (lift/set-down) or a big scan jump
        # triggers a RELOCALIZE window — stop navigating, rotate slowly to feed
        # slam_toolbox localization until it re-converges, then resume.
        self.declare_parameter("lift_accel_thresh", 4.0)   # m/s^2 off gravity
        self.declare_parameter("scan_jump_thresh", 0.6)    # m mean-range jump
        self.declare_parameter("relocalize_time_s", 5.0)   # spin-recover window
        self.declare_parameter("relocalize_wz", 0.5)       # rad/s recover spin
        # startup global localization: AMCL starts with particles spread over
        # the whole map, and only updates on motion — so spin until the
        # covariance says it has converged (or give up and say so).
        self.declare_parameter("auto_localize", True)
        self.declare_parameter("localize_cov_xy", 0.15)    # m^2, per axis
        self.declare_parameter("localize_cov_yaw", 0.15)   # rad^2
        self.declare_parameter("localize_timeout_s", 120.0)
        self.declare_parameter("localize_speed", 0.5)      # fraction of v_max

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.v_max, self.w_max = g("v_max"), g("w_max")
        self.tol = g("goal_tolerance")
        self.align = g("heading_align")
        self.k = g("steer_gain")
        self.front_cone = math.radians(g("front_cone_deg"))
        self.min_run = math.radians(g("min_run_deg"))
        self.slow_dist = g("slow_dist")
        self.pose_stale_s = g("pose_stale_s")
        self.obst_stale_s = g("obstacles_stale_s")
        self.map_frame = g("map_frame")
        self.base_frame = g("base_frame")
        self.lift_thresh = g("lift_accel_thresh")
        self.scan_jump = g("scan_jump_thresh")
        self.reloc_time = g("relocalize_time_s")
        self.reloc_wz = g("relocalize_wz")
        self.cov_xy = g("localize_cov_xy")
        self.cov_yaw = g("localize_cov_yaw")
        self.loc_timeout = g("localize_timeout_s")
        self.loc_speed = g("localize_speed")

        self.goal = None                # (x, y) in map frame
        self.sectors = None
        self.sectors_t = 0.0
        self.search_dir = 1.0
        self.state = ""
        self._disturb_t = None          # last lift/scan-jump disturbance time
        self._last_scan_mean = None
        # startup global localization
        self._localized = not g("auto_localize")
        self._amcl_cov = None           # (cov_xx, cov_yy, cov_yaw) newest pose
        self._loc_start_t = None
        self._reloc_sent_t = None
        self._reloc_futures = []        # keep refs — a GC'd future drops the call

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/goal", self._on_goal, latched)
        self.create_subscription(Sectors, "/obstacles", self._on_sectors, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_amcl_pose, 10)
        self._global_reloc = self.create_client(
            Empty, "/reinitialize_global_localization")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/behaviour/state", latched)

        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info("goal_navigator up -> /cmd_vel (waiting for /goal)")

    # ------------------------------------------------------------------ #
    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"new goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}) [{msg.header.frame_id}]")

    def _on_sectors(self, msg):
        self.sectors = msg
        self.sectors_t = self._now()

    def _on_amcl_pose(self, msg):
        """AMCL's pose covariance is how we know the particle cloud has
        collapsed onto one hypothesis (row-major 6x6: xx=0, yy=7, yaw=35)."""
        c = msg.pose.covariance
        self._amcl_cov = (c[0], c[7], c[35])

    def _on_imu(self, msg):
        """A lift or set-down shows up as an accel magnitude far from 1 g."""
        a = msg.linear_acceleration
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if abs(mag - _G) > self.lift_thresh:
            self._disturb_t = self._now()

    def _on_scan(self, msg):
        """A big jump in mean range = the world around us changed suddenly
        (carried elsewhere) — corroborates a kidnap."""
        vals = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if not vals:
            return
        mean = sum(vals) / len(vals)
        if self._last_scan_mean is not None \
                and abs(mean - self._last_scan_mean) > self.scan_jump:
            self._disturb_t = self._now()
        self._last_scan_mean = mean

    def _relocalizing(self):
        return (self._disturb_t is not None
                and self._now() - self._disturb_t < self.reloc_time)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        """map->base_link as (x, y, yaw), or None if unavailable/stale."""
        try:
            t = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _sector_free(self, bearing, half):
        """True if every sector overlapping [bearing-half, bearing+half] is
        FREE. Bearings outside the sector FOV return False (can't confirm)."""
        msg = self.sectors
        if msg is None or self._now() - self.sectors_t > self.obst_stale_s:
            return False
        n = len(msg.status)
        if n == 0:
            return False
        width = (msg.angle_max - msg.angle_min) / n
        lo, hi = bearing - half, bearing + half
        if lo < msg.angle_min or hi > msg.angle_max:
            return False
        i0 = max(0, int((lo - msg.angle_min) / width))
        i1 = min(n - 1, int((hi - msg.angle_min) / width))
        return all(msg.status[i] == Sectors.FREE for i in range(i0, i1 + 1))

    def _best_run(self):
        """(centre, width) of the widest FREE sector run >= min_run, or None.
        Mirrors local_planner._best_run so avoidance behaves identically."""
        msg = self.sectors
        if msg is None or self._now() - self.sectors_t > self.obst_stale_s:
            return None
        status = msg.status
        n = len(status)
        if n == 0:
            return None
        width = (msg.angle_max - msg.angle_min) / n
        best, run_start = None, None
        for i in range(n + 1):
            if i < n and status[i] == Sectors.FREE:
                if run_start is None:
                    run_start = i
                continue
            if run_start is not None:
                lo = msg.angle_min + run_start * width
                hi = msg.angle_min + i * width
                if best is None or hi - lo > best[1]:
                    best = ((lo + hi) / 2.0, hi - lo)
                run_start = None
        return best if best and best[1] >= self.min_run else None

    def _drive(self, cmd):
        """Publish a velocity command. A transient RMW publish failure (seen
        under the full navigate stack) used to raise straight out of the timer
        and kill the node, leaving the robot with no controller at all — log it
        and keep ticking instead. safety_gate stops the wheels either way if
        commands really do stop arriving."""
        try:
            self.pub.publish(cmd)
        except RuntimeError as e:                # rclpy RCLError subclasses this
            self.get_logger().warn(f"/cmd_vel publish failed: {e}",
                                   throttle_duration_sec=2.0)

    def _set_state(self, s):
        if s != self.state:
            self.state = s
            self.state_pub.publish(String(data=s))
            self.get_logger().info(f"state -> {s}")   # surfaces in console log

    def _converged(self):
        if self._amcl_cov is None:
            return False
        xx, yy, yaw = self._amcl_cov
        return xx < self.cov_xy and yy < self.cov_xy and yaw < self.cov_yaw

    def _localize_tick(self, cmd):
        """Find ourselves in the loaded map before accepting goals: ask AMCL to
        scatter particles map-wide, then drive around (see _wander) until the
        covariance collapses."""
        now = self._now()
        if self._loc_start_t is None:
            self._loc_start_t = now

        # AMCL publishes nothing at all until it has been given a starting
        # belief ("cannot publish a pose or update the transform"), so no
        # /amcl_pose yet means the scatter request has not landed. Keep asking
        # until it does — a single fire-and-forget call is easy to lose to
        # service discovery still settling right after activation.
        if self._amcl_cov is None:
            if not self._global_reloc.service_is_ready():
                self._set_state("LOCALIZING")    # amcl still coming up
                self._drive(cmd)
                return
            if self._reloc_sent_t is None or now - self._reloc_sent_t > 5.0:
                self._reloc_futures.append(
                    self._global_reloc.call_async(Empty.Request()))
                self._reloc_sent_t = now
                self.get_logger().info(
                    "global localization requested — spinning to converge")

        if self._converged():
            self._localized = True
            xx, yy, yaw = self._amcl_cov
            self.get_logger().info(
                f"localized in map (cov x={xx:.3f} y={yy:.3f} yaw={yaw:.3f}) "
                "— ready for goals")
            self._drive(cmd)            # stop the spin
            return

        if now - self._loc_start_t > self.loc_timeout:
            self._localized = True
            self.get_logger().warn(
                f"localization did not converge in {self.loc_timeout:.0f}s — "
                "accepting goals anyway, POSE MAY BE WRONG (drive it around, "
                "or re-enter NAVIGATE to retry)")
            self._drive(cmd)
            return

        self._set_state("LOCALIZING")
        self._wander(cmd)
        self._drive(cmd)

    def _wander(self, cmd):
        """Drive toward open space so AMCL's cloud can collapse. Rotating in
        place cannot do it: the C1 is a 360 deg lidar, so it already sees the
        whole room from where it stands and a turn returns the same ranges
        index-shifted — no new information. Only visiting different POSITIONS
        disambiguates. Same widest-free-run avoidance the AVOID state uses, so
        this never drives into a blocked sector (safety_gate underneath too)."""
        if self.sectors is None or self._now() - self.sectors_t > self.obst_stale_s:
            cmd.angular.z = self.reloc_wz          # no fresh scan -> just turn
            return
        run = self._best_run()
        if run is None:                            # nowhere free -> rotate-search
            cmd.angular.z = self.search_dir * self.w_max * 0.6
            return
        centre, _width = run
        self.search_dir = 1.0 if centre >= 0 else -1.0
        steer = float(_clip(self.k * centre, -self.w_max, self.w_max))
        if abs(centre) <= self.front_cone and self._sector_free(0.0, self.front_cone):
            cmd.linear.x = self.v_max * self.loc_speed
            cmd.angular.z = steer
        else:                                      # turn toward the opening first
            cmd.angular.z = steer or self.search_dir * 0.5

    def _tick(self):
        cmd = Twist()

        # we do not know where we are in the map yet -> localize first
        if not self._localized:
            self._localize_tick(cmd)
            return

        if self.goal is None:
            self._set_state("IDLE")
            self._drive(cmd)
            return

        # kidnap recovery: a recent lift/scan-jump -> stop navigating and rotate
        # slowly to feed AMCL motion, so its random-particle injection
        # (recovery_alpha_slow/fast) re-matches the map, then resume.
        if self._relocalizing():
            self._set_state("RELOCALIZE")
            cmd.angular.z = self.reloc_wz
            self._drive(cmd)
            return

        pose = self._pose()
        if pose is None:
            self._set_state("LOST")          # no localization / kidnapped
            self._drive(cmd)
            return

        rx, ry, ryaw = pose
        dx, dy = self.goal[0] - rx, self.goal[1] - ry
        dist = math.hypot(dx, dy)
        if dist <= self.tol:
            self._set_state("ARRIVED")
            self.goal = None                 # latch; a new /goal re-arms us
            self._drive(cmd)
            return

        bearing = _wrap(math.atan2(dy, dx) - ryaw)   # robot-frame angle to goal

        # not facing the goal -> rotate in place toward it (any bearing, incl.
        # behind us or outside the sector FOV; rotation always passes safety)
        if abs(bearing) > self.align:
            self._set_state("NAVIGATE")
            cmd.angular.z = float(_clip(self.k * bearing, -self.w_max, self.w_max))
            self.search_dir = 1.0 if bearing >= 0 else -1.0
            self._drive(cmd)
            return

        # aligned: drive toward the goal if the cone that way is FREE
        if self._sector_free(bearing, self.front_cone):
            self._set_state("NAVIGATE")
            cmd.linear.x = self.v_max * _clip(dist / self.slow_dist, 0.15, 1.0)
            cmd.angular.z = float(_clip(self.k * bearing, -self.w_max, self.w_max))
            self._drive(cmd)
            return

        # blocked toward the goal -> detour via the widest FREE run
        run = self._best_run()
        if run is None:                      # nowhere free -> rotate-search
            self._set_state("AVOID")
            cmd.angular.z = self.search_dir * self.w_max * 0.6
            self._drive(cmd)
            return
        self._set_state("AVOID")
        centre, width = run
        self.search_dir = 1.0 if centre >= 0 else -1.0
        lo, hi = centre - width / 2.0, centre + width / 2.0
        if lo <= -self.front_cone and hi >= self.front_cone:
            cmd.linear.x = self.v_max * 0.5
            cmd.angular.z = float(_clip(self.k * centre, -self.w_max, self.w_max))
        else:
            cmd.angular.z = (float(_clip(self.k * centre, -self.w_max, self.w_max))
                             or self.search_dir * 0.5)
        self._drive(cmd)


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
