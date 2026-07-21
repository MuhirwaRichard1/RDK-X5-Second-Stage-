#!/usr/bin/env python3
"""
08 — safety_gate proximity ring: swath, not half-plane.

The ring stops motion TOWARD an intruder inside the stop circle. It used to
decide "toward" by half-plane (|bearing| <= 90 deg), which counts anything
beside the robot as ahead: parked 24 cm from a wall, forward was vetoed and the
robot could only turn — unable to escape, and (360 deg lidar) unable to
localize, since turning on the spot yields no new information. It now tests the
corridor the robot would actually sweep, +/- stop_halfwidth_m of the centre
line.

Feeds synthetic scans straight into SafetyGate._on_scan and checks
_scan_unsafe(). No motors, no other nodes — but run it isolated anyway so its
node cannot join the live graph:

    ROS_DOMAIN_ID=42 python3 bench_tests/08_safety_gate_swath_test.py

NOTE the baseline: every scan is filled with far returns (a ~3 m room). A cone
with no finite returns fail-safes to BLOCKED by design, so a scan containing
only the one obstacle blocks for that reason and tells you nothing about the
swath logic.
"""

import math
import sys
import time

import rclpy
from sensor_msgs.msg import LaserScan

from navbot_drive.safety_gate import SafetyGate

_N = 720                      # C1 delivers 720 points/rev


def scan(points, background=3.0):
    """A full revolution of `background` returns, with `points` painted in as
    (bearing_rad, range_m) in the ROBOT frame (yaw_offset_deg defaults to 0)."""
    m = LaserScan()
    m.angle_min, m.angle_max = -math.pi, math.pi
    m.angle_increment = (m.angle_max - m.angle_min) / _N
    ranges = [background] * _N
    for ang, r in points:
        i = int((ang - m.angle_min) / m.angle_increment) % _N
        ranges[i] = r
    m.ranges = ranges
    return m


def main():
    rclpy.init()
    gate = SafetyGate()

    cases = [
        # the failure this fix is for: close, but beside us, path ahead clear
        ("24cm at 76 deg (the trap: beside robot)", [(math.radians(76), 0.243)], False),
        ("24cm at 90 deg (directly beside)", [(math.radians(90), 0.24)], False),
        ("24cm at -76 deg (other side)", [(math.radians(-76), 0.243)], False),
        # still must stop for anything genuinely in the way
        ("25cm straight ahead", [(0.0, 0.25)], True),
        ("25cm at 30 deg (diagonally ahead)", [(math.radians(30), 0.25)], True),
        ("open room", [], False),
    ]

    results = []
    for name, points, want_blocked in cases:
        gate._on_scan(scan(points))
        gate.scan_t = time.monotonic()          # mark the scan fresh
        got = gate._scan_unsafe()
        ok = got == want_blocked
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: forward blocked={got} "
              f"(want {want_blocked})")

    gate.destroy_node()
    rclpy.try_shutdown()

    n_pass = sum(results)
    print(f"\n{n_pass}/{len(results)} scenarios passed")
    sys.exit(0 if n_pass == len(results) else 1)


if __name__ == "__main__":
    main()
