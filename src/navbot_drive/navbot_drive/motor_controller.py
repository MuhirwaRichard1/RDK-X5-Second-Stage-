#!/usr/bin/env python3
"""
motor_controller — open-loop differential drive for the RDK X5 Tri-Cam NavBot.

Subscribes  : /cmd_vel_safe  (geometry_msgs/Twist)   <- output of safety_gate
Drives      : L298N  ENA/ENB = hardware PWM (speed),  IN1-4 = GPIO (direction)
Feedback    : NONE (no wheel encoders). Speed comes from a measured
              duty<->velocity lookup table (config/drive_lut.yaml), and the
              motor loop is closed *visually* by visual_odometry upstream.

Two safety behaviours are built in:
  * Dead-man : if no /cmd_vel_safe arrives within `cmd_timeout` seconds the
               motors coast to a stop (does not depend on the AI stack).
  * Clean stop on shutdown: PWM stopped, GPIO released.

Hobot.GPIO quirk (seen in servo_sweep.py): PWM.start() gates the sysfs "enable"
write on the *previous* duty (still 0 right after __init__), so the channel
never turns on. We force enable by writing "1" to pin_info[pin].pwm_enable.

Pins are BOARD numbering. ENA/ENB default to pins 32/33 (PWM6/PWM7 on the
34170000 controller). That controller needs `dtoverlay=dtoverlay_pwm3` in
/boot/config.txt (disables PWM on pins 18/29/31/37); verify with
`ls /sys/class/pwm/` after reboot.
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import Hobot.GPIO as GPIO
import yaml


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # ---- parameters (override in config/params.yaml or launch) ----
        self.declare_parameter("ena_pin", 32)      # BOARD pin, PWM6, motor A (left) speed
        self.declare_parameter("enb_pin", 33)      # BOARD pin, PWM7, motor B (right) speed
        self.declare_parameter("in1_pin", 16)      # motor A direction
        self.declare_parameter("in2_pin", 18)
        self.declare_parameter("in3_pin", 22)      # motor B direction
        self.declare_parameter("in4_pin", 36)
        self.declare_parameter("pwm_freq_hz", 1000.0)
        self.declare_parameter("wheel_separation", 0.15)   # m, centre-to-centre
        self.declare_parameter("cmd_timeout", 0.2)         # s, dead-man window
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("lut_path", "config/drive_lut.yaml")
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        # This chassis' motor harness is cross-wired (L/R channels swapped AND
        # polarity reversed), which inverts linear.x but leaves angular.z
        # correct. Negating v compensates exactly; do NOT "fix" it with
        # invert_left/right — that would un-invert v but flip turning.
        self.declare_parameter("invert_linear", True)

        g = lambda n: self.get_parameter(n).value
        self.ena, self.enb = g("ena_pin"), g("enb_pin")
        self.in1, self.in2 = g("in1_pin"), g("in2_pin")
        self.in3, self.in4 = g("in3_pin"), g("in4_pin")
        self.wheel_sep = float(g("wheel_separation"))
        self.cmd_timeout = float(g("cmd_timeout"))
        self.invert_left = bool(g("invert_left"))
        self.invert_right = bool(g("invert_right"))
        self.invert_linear = bool(g("invert_linear"))
        pwm_freq = float(g("pwm_freq_hz"))

        # ---- duty<->velocity calibration (encoderless) ----
        self._load_lut(g("lut_path"))

        # ---- GPIO / PWM setup ----
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        for pin in (self.in1, self.in2, self.in3, self.in4):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        self.pwm_a = GPIO.PWM(self.ena, pwm_freq)
        self.pwm_b = GPIO.PWM(self.enb, pwm_freq)
        self.pwm_a.start(0.0)
        self.pwm_b.start(0.0)
        self._force_pwm_enable(self.ena)
        self._force_pwm_enable(self.enb)

        # ---- ROS plumbing ----
        self._target = (0.0, 0.0)          # (linear.x, angular.z)
        self._last_cmd = self.get_clock().now()
        self.sub = self.create_subscription(
            Twist, "/cmd_vel_safe", self._on_cmd, 10)
        self.timer = self.create_timer(1.0 / float(g("control_rate_hz")),
                                       self._control_step)

        self.get_logger().info(
            f"motor_controller up: ENA={self.ena} ENB={self.enb} "
            f"IN=({self.in1},{self.in2},{self.in3},{self.in4}) "
            f"wheel_sep={self.wheel_sep} m, dead-man={self.cmd_timeout}s, "
            f"max_v={self.max_v:.2f} m/s")

    # ------------------------------------------------------------------ #
    def _load_lut(self, path):
        """Load duty<->velocity table; build sorted arrays for interpolation."""
        if not os.path.isabs(path) and not os.path.exists(path):
            # tolerate running from repo root or package dir
            alt = os.path.join(os.getcwd(), path)
            path = alt if os.path.exists(alt) else path
        try:
            with open(path) as f:
                cfg = yaml.safe_load(f)
            rows = sorted(cfg["lut"], key=lambda r: r["v"])
            self._v = [float(r["v"]) for r in rows]
            self._duty = [float(r["duty"]) for r in rows]   # fraction 0..1
            self.trim_l = float(cfg.get("trim", {}).get("left", 1.0))
            self.trim_r = float(cfg.get("trim", {}).get("right", 1.0))
            self.max_v = float(cfg.get("max_cmd_v", self._v[-1]))
            self.get_logger().info(f"loaded drive LUT from {path}")
        except Exception as e:                                # noqa: BLE001
            # Safe fallback: linear 0..1 duty over 0..0.4 m/s, no trim.
            self.get_logger().warn(
                f"could not load LUT ({e}); using linear fallback")
            self._v = [0.0, 0.4]
            self._duty = [0.0, 1.0]
            self.trim_l = self.trim_r = 1.0
            self.max_v = 0.4

    def _force_pwm_enable(self, pin):
        """Work around Hobot.GPIO PWM.start() not enabling the channel."""
        try:
            with open(GPIO.pin_info[pin].pwm_enable, "w") as f:
                f.write("1")
        except (KeyError, AttributeError, FileNotFoundError, OSError) as e:
            self.get_logger().error(
                f"no PWM channel for BOARD pin {pin} ({e}). "
                f"Check `ls /sys/class/pwm/` and RDK_X5_Peripherals.md.")

    # ------------------------------------------------------------------ #
    def _on_cmd(self, msg: Twist):
        self._target = (msg.linear.x, msg.angular.z)
        self._last_cmd = self.get_clock().now()

    def _control_step(self):
        # dead-man: stale command -> coast to stop
        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        v, w = (0.0, 0.0) if age > self.cmd_timeout else self._target
        if self.invert_linear:
            v = -v

        # differential-drive inverse kinematics -> wheel linear speeds (m/s)
        v_left = v - w * self.wheel_sep / 2.0
        v_right = v + w * self.wheel_sep / 2.0
        if self.invert_left:
            v_left = -v_left
        if self.invert_right:
            v_right = -v_right

        duty_l = self._speed_to_duty(abs(v_left)) * self.trim_l
        duty_r = self._speed_to_duty(abs(v_right)) * self.trim_r

        self._drive(self.in1, self.in2, self.pwm_a, v_left >= 0, duty_l)
        self._drive(self.in3, self.in4, self.pwm_b, v_right >= 0, duty_r)

    def _speed_to_duty(self, speed):
        """Map |wheel speed| (m/s) -> PWM duty percent via the LUT.

        A near-zero request must coast (0 %), NOT the LUT's v=0 row — that row
        is the *minimum duty to start moving* (stall threshold), so returning it
        here would energize the motors while "stopped".
        """
        if speed <= 1e-3:
            return 0.0
        speed = min(speed, self.max_v)
        duty_frac = _interp(speed, self._v, self._duty)     # 0..1
        return max(0.0, min(100.0, duty_frac * 100.0))

    @staticmethod
    def _drive(in_a, in_b, pwm, forward, duty_pct):
        if duty_pct <= 0.0:                 # coast (both low) — gentler than brake
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.LOW)
            pwm.ChangeDutyCycle(0.0)
            return
        GPIO.output(in_a, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(in_b, GPIO.LOW if forward else GPIO.HIGH)
        pwm.ChangeDutyCycle(duty_pct)

    def shutdown(self):
        try:
            self.pwm_a.ChangeDutyCycle(0.0)
            self.pwm_b.ChangeDutyCycle(0.0)
            self.pwm_a.stop()
            self.pwm_b.stop()
        finally:
            GPIO.cleanup()


def _interp(x, xs, ys):
    """Linear interpolation with flat clamping at both ends (no numpy dep)."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
