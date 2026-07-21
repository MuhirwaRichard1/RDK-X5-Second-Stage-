#!/usr/bin/env python3
"""Step 7 - goal_navigator behavioral test (no hardware, no motors).

Spins the REAL navbot_navigation goal_navigator against a fake graph:
  * a TF broadcaster feeding map->base_link (the robot pose),
  * /obstacles (navbot_msgs/Sectors) with a chosen free/blocked pattern,
  * a /goal (map frame),
and captures /cmd_vel to assert the drive decision in each scenario.

Run:      source /opt/tros/humble/setup.bash && source install/setup.bash
          python3 bench_tests/07_goal_navigator_test.py
Pass if:  all scenarios print PASS (exit 0). safety_gate is NOT in the loop
          here — this checks the planner's intent only.
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import Imu
from navbot_msgs.msg import Sectors
from navbot_navigation.goal_navigator import GoalNavigator

N = 26                                   # sectors, matches scan_sectors default
A_MIN, A_MAX = -math.radians(130), math.radians(130)


def all_free():
    return [Sectors.FREE] * N


def front_blocked():
    """Block the sectors inside ~±25° (the forward cone)."""
    s = all_free()
    for i in range(N):
        centre = A_MIN + (i + 0.5) * (A_MAX - A_MIN) / N
        if abs(centre) <= math.radians(25):
            s[i] = Sectors.BLOCKED
    return s


class Harness(Node):
    def __init__(self):
        super().__init__("goal_nav_test_harness")
        self.pose = None                 # (x, y, yaw) or None -> don't publish TF
        self.status = all_free()
        self.last_cmd = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._obst = self.create_publisher(Sectors, "/obstacles", 10)
        self._goal = self.create_publisher(PoseStamped, "/goal", latched)
        self._imu = self.create_publisher(Imu, "/imu/data", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self._tf = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.05, self._tick)      # 20 Hz feed

    def _on_cmd(self, msg):
        self.last_cmd = (msg.linear.x, msg.angular.z)

    def _tick(self):
        m = Sectors()
        m.header.stamp = self.get_clock().now().to_msg()
        m.angle_min, m.angle_max = float(A_MIN), float(A_MAX)
        m.status = bytes(self.status)
        m.free_fraction = [1.0 if s == Sectors.FREE else 0.0 for s in self.status]
        self._obst.publish(m)
        if self.pose is not None:
            x, y, yaw = self.pose
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self._tf.sendTransform(t)

    def set_goal(self, x, y):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        g.pose.orientation.w = 1.0
        self._goal.publish(g)

    def inject_lift(self):
        """Publish one IMU sample with accel far from 1 g (a lift/set-down)."""
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.linear_acceleration.z = 2.0        # ~7.8 m/s^2 off gravity
        self._imu.publish(m)


def main():
    rclpy.init()
    nav = GoalNavigator()
    h = Harness()
    ex = MultiThreadedExecutor()
    ex.add_node(nav)
    ex.add_node(h)
    threading.Thread(target=ex.spin, daemon=True).start()

    def run(name, pose, status, goal, check, settle=1.0):
        h.pose = pose
        h.status = status
        h.last_cmd = None
        if goal is not None:
            h.set_goal(*goal)
        time.sleep(settle)
        vx, az = h.last_cmd if h.last_cmd else (None, None)
        ok = h.last_cmd is not None and check(vx, az)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: vx={vx}, az={az}")
        return ok

    results = []
    # LOST: goal set but no map->base_link TF -> must stop.
    results.append(run("LOST (no TF -> stop)", None, all_free(), (2.0, 0.0),
                       lambda vx, az: abs(vx) < 1e-6 and abs(az) < 1e-6))
    # FORWARD: goal straight ahead, clear -> drive forward, little turn.
    results.append(run("FORWARD (goal ahead, clear)", (0.0, 0.0, 0.0),
                       all_free(), (2.0, 0.0),
                       lambda vx, az: vx > 0.05 and abs(az) < 0.2))
    # ROTATE-LEFT: goal 90 deg left -> turn left in place.
    results.append(run("ROTATE to goal on the left", (0.0, 0.0, 0.0),
                       all_free(), (0.0, 2.0),
                       lambda vx, az: az > 0.2 and vx < 0.02))
    # ROTATE-BEHIND: goal behind -> turn in place (never reverse).
    results.append(run("ROTATE to goal behind", (0.0, 0.0, 0.0),
                       all_free(), (-2.0, 0.0),
                       lambda vx, az: abs(az) > 0.2 and vx < 0.02))
    # AVOID: goal ahead but forward cone blocked -> don't drive in, steer away.
    results.append(run("AVOID (goal ahead but blocked)", (0.0, 0.0, 0.0),
                       front_blocked(), (2.0, 0.0),
                       lambda vx, az: vx < 0.12 and abs(az) > 0.1))
    # ARRIVED: already at the goal -> stop.
    results.append(run("ARRIVED (within tolerance)", (0.0, 0.0, 0.0),
                       all_free(), (0.05, 0.0),
                       lambda vx, az: abs(vx) < 1e-6 and abs(az) < 1e-6))

    # KIDNAP: navigating a clear goal, then a lift -> RELOCALIZE. The robot
    # must DRIVE, not spin: the C1 sees 360 deg, so turning on the spot gives
    # AMCL no new information and the scatter never collapses. Open space all
    # round, so it should roll forward roughly straight, capped below v_max.
    h.pose = (0.0, 0.0, 0.0)
    h.status = all_free()
    h.set_goal(2.0, 0.0)
    time.sleep(0.5)                      # driving forward
    h.last_cmd = None
    h.inject_lift()
    time.sleep(0.5)                      # inside the relocalize window
    vx, az = h.last_cmd if h.last_cmd else (None, None)
    ok = (h.last_cmd is not None
          and 0.02 < vx < 0.22           # translating, but slower than v_max
          and abs(az) < 0.4)             # gentle: fast turns lose icp odometry
    print(f"[{'PASS' if ok else 'FAIL'}] KIDNAP (lift -> relocalize by driving): "
          f"vx={vx}, az={az}")
    results.append(ok)

    ex.shutdown()
    nav.destroy_node()
    h.destroy_node()
    rclpy.try_shutdown()

    n_pass = sum(results)
    print(f"\n{n_pass}/{len(results)} scenarios passed")
    sys.exit(0 if n_pass == len(results) else 1)


if __name__ == "__main__":
    main()
