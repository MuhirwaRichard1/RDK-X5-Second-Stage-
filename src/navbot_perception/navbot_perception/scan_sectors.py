#!/usr/bin/env python3
"""
scan_sectors — RPLidar C1 /scan -> /obstacles sector classification.

Subscribes : /scan (sensor_msgs/LaserScan, from sllidar_node, 10 Hz)
Publishes  : /obstacles (navbot_msgs/Sectors) — same contract the retired
             camera pipelines (obstacle_fusion / depth_freespace) published,
             so local_planner, safety_gate's sector check, the console HUD
             fan and the agent's mode-activation probe all keep working
             unchanged.

Each scan is binned into `sector_deg`-wide sectors spanning
[angle_min_deg, angle_max_deg] in the ROBOT frame (REP-103: 0 = straight
ahead, CCW positive). `yaw_offset_deg` maps laser-frame ray angles to robot
bearings — keep it consistent with the base_link->laser static transform.

Classification per sector:
  BLOCKED : nearest finite return closer than `block_range_m`
  FREE    : nearest return farther, or only no-return rays (inf = nothing
            within the C1's 16 m — open space indoors)
  UNKNOWN : no usable rays at all (only self-hits under `min_range_m`) —
            consumers treat UNKNOWN as blocked, per the Sectors contract
free_fraction = nearest distance normalised by `far_range_m` (0 UNKNOWN).

Unlike the camera pipelines this costs no BPU and works in the dark.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from navbot_msgs.msg import Sectors


class ScanSectors(Node):
    def __init__(self):
        super().__init__("scan_sectors")

        self.declare_parameter("angle_min_deg", -130.0)   # rightmost edge
        self.declare_parameter("angle_max_deg", 130.0)    # leftmost edge
        self.declare_parameter("sector_deg", 10.0)
        self.declare_parameter("block_range_m", 0.50)     # nearer -> BLOCKED
        self.declare_parameter("far_range_m", 2.0)        # free_fraction scale
        self.declare_parameter("min_range_m", 0.12)       # under -> self-hit, drop
        self.declare_parameter("yaw_offset_deg", 0.0)     # laser -> robot frame

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.a0 = math.radians(float(g("angle_min_deg")))
        self.a1 = math.radians(float(g("angle_max_deg")))
        self.n = max(1, round(math.degrees(self.a1 - self.a0)
                              / float(g("sector_deg"))))
        self.block = float(g("block_range_m"))
        self.far = float(g("far_range_m"))
        self.min_r = float(g("min_range_m"))
        self.yaw_off = math.radians(float(g("yaw_offset_deg")))

        self.pub = self.create_publisher(Sectors, "/obstacles", 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"scan_sectors up: {self.n} x {math.degrees(self.a1 - self.a0) / self.n:.0f}° "
            f"sectors over [{math.degrees(self.a0):.0f}°, {math.degrees(self.a1):.0f}°], "
            f"blocked < {self.block:.2f} m")

    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        bearing = (msg.angle_min
                   + np.arange(len(ranges)) * msg.angle_increment
                   + self.yaw_off + math.pi) % (2.0 * math.pi) - math.pi

        width = (self.a1 - self.a0) / self.n
        sector = np.floor((bearing - self.a0) / width).astype(int)
        in_span = (sector >= 0) & (sector < self.n)
        finite = np.isfinite(ranges) & (ranges >= self.min_r)
        noret = ~np.isfinite(ranges)                  # inf = nothing in range

        status = [Sectors.UNKNOWN] * self.n
        fraction = [0.0] * self.n
        for i in range(self.n):
            m = in_span & (sector == i)
            hits = ranges[m & finite]
            if hits.size:
                d = float(hits.min())
                status[i] = Sectors.BLOCKED if d < self.block else Sectors.FREE
                fraction[i] = min(1.0, d / self.far)
            elif np.any(m & noret):                   # only no-returns: open
                status[i] = Sectors.FREE
                fraction[i] = 1.0

        out = Sectors()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "base_link"
        out.angle_min, out.angle_max = float(self.a0), float(self.a1)
        out.status = bytes(status)
        out.free_fraction = fraction
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSectors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
