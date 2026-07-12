#!/usr/bin/env python3
"""
local_planner — reactive free-space navigation from /obstacles -> /cmd_vel.

Subscribes : /obstacles (navbot_msgs/Sectors, from scan_sectors)
Publishes  : /cmd_vel (geometry_msgs/Twist) at `rate_hz` (default 20 Hz)

Policy (per PROPOSAL): never command forward motion into a BLOCKED or
UNKNOWN direction. Finds the widest contiguous FREE run of sectors; if it
covers the forward cone, drive forward while steering toward the run's
centre; otherwise rotate toward it. No FREE run anywhere -> rotate-in-place
search (keeps the last direction to avoid dithering). Stale /obstacles
(> stale_s) -> publish zero Twist.

The Twist goes to safety_gate (-> /cmd_vel_safe -> motor_controller), which
adds the scan forward stop and E-stop on top.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from navbot_msgs.msg import Sectors


class LocalPlanner(Node):
    def __init__(self):
        super().__init__("local_planner")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("v_max", 0.25)          # m/s forward
        self.declare_parameter("w_max", 1.2)           # rad/s
        self.declare_parameter("steer_gain", 1.5)      # w per rad of bearing error
        self.declare_parameter("front_cone_deg", 20.0) # must be FREE to drive
        self.declare_parameter("min_run_deg", 25.0)    # narrower runs are ignored
        self.declare_parameter("stale_s", 0.5)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.v_max, self.w_max = g("v_max"), g("w_max")
        self.k = g("steer_gain")
        self.front_cone = np.radians(g("front_cone_deg"))
        self.min_run = np.radians(g("min_run_deg"))
        self.stale_s = g("stale_s")

        self.sectors = None
        self.rx_time = 0.0
        self.search_dir = 1.0                          # +1 = left (REP-103)

        self.create_subscription(Sectors, "/obstacles", self._on_sectors, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info("local_planner up -> /cmd_vel")

    def _on_sectors(self, msg):
        self.sectors = msg
        self.rx_time = time.time()

    def _best_run(self, msg):
        """-> (centre bearing, width) of the widest FREE sector run, or None."""
        status = np.asarray(msg.status)
        n = len(status)
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

    def _tick(self):
        cmd = Twist()
        msg = self.sectors
        if msg is None or time.time() - self.rx_time > self.stale_s:
            self.pub.publish(cmd)                       # no data -> stop
            return

        run = self._best_run(msg)
        if run is None:                                 # nowhere free -> search
            cmd.angular.z = self.search_dir * self.w_max * 0.6
            self.pub.publish(cmd)
            return

        centre, width = run
        self.search_dir = 1.0 if centre >= 0 else -1.0
        # forward only if the whole forward cone lies inside the free run
        lo, hi = centre - width / 2.0, centre + width / 2.0
        if lo <= -self.front_cone and hi >= self.front_cone:
            cmd.linear.x = self.v_max
            cmd.angular.z = float(np.clip(self.k * centre,
                                          -self.w_max, self.w_max))
        else:                                           # rotate toward the run
            cmd.angular.z = float(np.clip(self.k * centre, -self.w_max,
                                          self.w_max)) or self.search_dir * 0.5
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
