#!/usr/bin/env python3
"""
safety_gate — the only writer of /cmd_vel_safe.

Subscribes : /cmd_vel (geometry_msgs/Twist, from local_planner or teleop)
Publishes  : /cmd_vel_safe (geometry_msgs/Twist) — consumed by motor_controller
             /estop_state (std_msgs/Bool, latched) — E-stop readback for operator UIs
             /range_forward (sensor_msgs/Range) — TF-Luna range, NaN when unreadable
Service    : /estop (std_srvs/SetBool)  data=true engages the E-stop

Two independent protections on top of whatever the planner commands:
  * TF-Luna forward range (I2C5 @ 0x10, polled in-process): closer than
    `stop_cm` -> forward velocity is clamped to 0 (reverse/rotate still pass,
    so the robot can extract itself). Sensor unreadable -> fail-safe: forward
    is blocked.
  * E-stop: while engaged every command is zeroed.

motor_controller's own dead-man (cmd_timeout) remains the last line: if this
node dies, motors coast to a stop.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool

from smbus2 import SMBus, i2c_msg

TFLUNA_ADDR = 0x10


class SafetyGate(Node):
    def __init__(self):
        super().__init__("safety_gate")

        self.declare_parameter("stop_cm", 30.0)
        self.declare_parameter("i2c_bus", 5)
        self.declare_parameter("min_amp", 100)     # TF-Luna signal strength floor
        self.declare_parameter("lidar_rate_hz", 20.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stop_cm = g("stop_cm")
        self.min_amp = g("min_amp")

        self.bus = SMBus(g("i2c_bus"))
        self.estop = False
        self.lidar_cm = None                       # None = unreadable

        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = self.create_publisher(Bool, "/estop_state", latched)
        self.pub_estop.publish(Bool(data=False))
        self.pub_range = self.create_publisher(Range, "/range_forward", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_service(SetBool, "/estop", self._on_estop)
        self.create_timer(1.0 / g("lidar_rate_hz"), self._read_lidar)
        self.get_logger().info(
            f"safety_gate up: stop at {self.stop_cm:.0f} cm, /estop ready")

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

    def _on_estop(self, req, resp):
        self.estop = req.data
        self.pub_estop.publish(Bool(data=self.estop))
        self.get_logger().warn(f"E-STOP {'ENGAGED' if self.estop else 'released'}")
        if self.estop:
            self.pub.publish(Twist())              # zero immediately
        resp.success = True
        resp.message = "engaged" if self.estop else "released"
        return resp

    def _on_cmd(self, msg):
        out = Twist()
        if not self.estop:
            out.angular = msg.angular
            out.linear = msg.linear
            forward_unsafe = (self.lidar_cm is None
                              or self.lidar_cm < self.stop_cm)
            if msg.linear.x > 0.0 and forward_unsafe:
                out.linear.x = 0.0                 # block forward, keep rotation
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
