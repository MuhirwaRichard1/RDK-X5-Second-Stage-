#!/usr/bin/env python3
"""
dr_odom — dead-reckoning odometry for the encoderless NavBot.

Visual odometry drops out on dark/close/textureless views and every dropout
fragments the SLAM graph. This node provides an odometry backbone that cannot
drop out, from the two signals that survive those views:

  translation : commanded velocity (/cmd_vel_safe linear.x). The motor
                controller executes commands open-loop through a calibrated
                duty<->velocity LUT, so commanded ~= executed while motors are
                armed. Replicates the controller's dead-man: a command older
                than cmd_timeout means the motors have coasted to a stop.
  heading     : IMU gyro z, bias-corrected. The gyro is a real measurement and
                far more trustworthy than commanded angular velocity; bias is
                learned (slow EMA) only while the robot is commanded still.

Integration happens per IMU sample using IMU header stamps for dt, so the same
code is correct live and under `ros2 bag play --clock` (sim time). Pose is
planar (x, y, yaw); z/roll/pitch stay 0 — pair with Reg/Force3DoF.

Publishes:
  /odom_dr             nav_msgs/Odometry  (frame odom -> base_link)
  TF odom -> base_link (unless publish_tf:=false)

CAVEAT: only meaningful while the motors actually execute commands (agent mode
manual/auto with motors armed). With motors disarmed, nonzero commands would
integrate phantom motion. Translation drifts with wheel slip and LUT error —
RTAB-Map's loop closures are the correction layer on top.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class DrOdom(Node):
    def __init__(self):
        super().__init__("dr_odom")

        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom_dr")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # mirror motor_controller: stale command => motors have coasted to stop
        self.declare_parameter("cmd_timeout", 0.2)
        self.declare_parameter("max_v", 0.4)            # agent v_max clamp
        self.declare_parameter("publish_div", 6)        # ~190 Hz IMU -> ~32 Hz odom
        self.declare_parameter("still_time", 1.0)       # s commanded-still before bias learning
        self.declare_parameter("bias_alpha", 0.02)      # EMA weight per IMU sample
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.cmd_timeout = float(g("cmd_timeout"))
        self.max_v = float(g("max_v"))
        self.publish_div = int(g("publish_div"))
        self.still_time = float(g("still_time"))
        self.bias_alpha = float(g("bias_alpha"))

        # state
        self.x = self.y = self.yaw = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.cmd_stamp = None               # ROS time (s) of last command
        self.still_since = None             # ROS time (s) commanded-still began
        self.gyro_bias = 0.0
        self.bias_samples = 0
        self.last_imu_t = None
        self._n = 0

        self.create_subscription(Twist, g("cmd_topic"), self._on_cmd, 10)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, g("imu_topic"), self._on_imu, qos)
        self.pub = self.create_publisher(Odometry, g("odom_topic"), 10)
        self.tf = TransformBroadcaster(self) if bool(g("publish_tf")) else None

        self.get_logger().info(
            f"dr_odom up: cmd={g('cmd_topic')} imu={g('imu_topic')} -> "
            f"{g('odom_topic')} ({self.odom_frame}->{self.base_frame})")

    # ------------------------------------------------------------------ #
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd_v = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.cmd_w = msg.angular.z
        self.cmd_stamp = self._now_s()
        if self.cmd_v == 0.0 and self.cmd_w == 0.0:
            if self.still_since is None:
                self.still_since = self.cmd_stamp
        else:
            self.still_since = None

    def _commanded_v(self):
        """Executed linear velocity per the motor controller's dead-man."""
        if self.cmd_stamp is None or self._now_s() - self.cmd_stamp > self.cmd_timeout:
            return 0.0
        return self.cmd_v

    def _on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        if self.last_imu_t is None:
            self.last_imu_t = t
            self.gyro_bias = gz             # first sample seeds the bias
            self.bias_samples = 1
            return
        dt = t - self.last_imu_t
        self.last_imu_t = t
        if dt <= 0.0 or dt > 0.5:
            return                          # clock jump (bag restart) — skip

        # learn gyro bias only while ACTIVELY commanded still (zero commands
        # flowing — the teleop/safety-gate heartbeat publishes zeros at rest).
        # Exception: boot calibration before any command arrives, capped at the
        # fast-convergence window so a command-less replay can't absorb real
        # rotation into the bias.
        still = (self.still_since is not None
                 and self._now_s() - self.still_since > self.still_time) \
            or (self.cmd_stamp is None and self.bias_samples < 200)
        if still:
            if self.bias_samples < 200:     # fast initial convergence
                self.bias_samples += 1
                self.gyro_bias += (gz - self.gyro_bias) / self.bias_samples
            else:
                self.gyro_bias += self.bias_alpha * (gz - self.gyro_bias)

        wz = gz - self.gyro_bias
        v = self._commanded_v()
        # midpoint integration: translate along the half-step heading
        self.x += v * math.cos(self.yaw + 0.5 * wz * dt) * dt
        self.y += v * math.sin(self.yaw + 0.5 * wz * dt) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt),
                              math.cos(self.yaw + wz * dt))

        self._n += 1
        if self._n % self.publish_div == 0:
            self._publish(msg.header.stamp, v, wz)

    def _publish(self, stamp, v, wz):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        # generous translation uncertainty (open-loop commanded v), tight-ish
        # yaw (gyro); z/roll/pitch effectively fixed (planar).
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.25
        pc[14] = pc[21] = pc[28] = 1e-3
        pc[35] = 0.05
        o.pose.covariance = pc
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = wz
        tc = [0.0] * 36
        tc[0] = 0.01
        tc[7] = tc[14] = 1e-3
        tc[21] = tc[28] = 1e-3
        tc[35] = 4e-4
        o.twist.covariance = tc
        self.pub.publish(o)

        if self.tf is not None:
            tfm = TransformStamped()
            tfm.header.stamp = stamp
            tfm.header.frame_id = self.odom_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = self.x
            tfm.transform.translation.y = self.y
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf.sendTransform(tfm)


def main(args=None):
    rclpy.init(args=args)
    node = DrOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
