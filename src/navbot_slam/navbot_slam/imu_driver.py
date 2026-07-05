#!/usr/bin/env python3
"""
imu_driver — MPU6050 (I2C5 @ 0x68) -> /imu/data for the RDK X5 Tri-Cam NavBot.

Publishes   : /imu/data  (sensor_msgs/Imu) at `rate_hz` (default 100 Hz).
Orientation : NOT estimated here (orientation_covariance[0] = -1 by REP-145
              convention); the SLAM back-end integrates the raw rates itself.

Startup calibration (robot MUST be still for the first ~2 s):
  * gyro bias  — mean of `calib_samples` readings, subtracted from every sample.
  * accel scale — this robot's MPU6050 is a clone whose accelerometer reads
    gravity as ~0.55 g even with ACCEL_CONFIG at +/-2 g (verified 2026-07-06),
    so the datasheet LSB/g constant cannot be trusted. With `auto_scale_accel`
    (default on) the measured gravity magnitude is rescaled to exactly 1 g;
    on a genuine chip the factor lands near 1.0 and this is a no-op.

Wiring: TF-Luna shares the bus (0x10) — see scripts/test_i2c_sensors.py for a
bench check of both. I2C5 = header pins 3 (SDA) / 5 (SCL), /dev/i2c-5.
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from smbus2 import SMBus

G = 9.80665                 # m/s^2 per g
ACCEL_LSB_PER_G = 16384.0   # +/-2 g full scale
GYRO_LSB_PER_DPS = 131.0    # +/-250 deg/s full scale

# MPU6050 registers
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75
REG_DATA = 0x3B             # ACCEL_XOUT_H .. GYRO_ZOUT_L (14 bytes)


class ImuDriver(Node):
    def __init__(self):
        super().__init__("imu_driver")

        self.declare_parameter("i2c_bus", 5)
        self.declare_parameter("i2c_addr", 0x68)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("calib_samples", 200)
        self.declare_parameter("auto_scale_accel", True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.addr = g("i2c_addr")
        self.frame_id = g("frame_id")
        rate_hz = g("rate_hz")

        self.bus = SMBus(g("i2c_bus"))
        self._init_chip()

        self.gyro_bias = (0.0, 0.0, 0.0)
        self.accel_scale = 1.0
        self._calibrate(g("calib_samples"), g("auto_scale_accel"))

        self.pub = self.create_publisher(Imu, "/imu/data", 50)
        self.msg = Imu()
        self.msg.header.frame_id = self.frame_id
        # No orientation estimate (REP-145): first element = -1.
        self.msg.orientation_covariance[0] = -1.0
        # Conservative static covariances; SLAM treats these as priors.
        for i in (0, 4, 8):
            self.msg.angular_velocity_covariance[i] = (0.02) ** 2   # (rad/s)^2
            self.msg.linear_acceleration_covariance[i] = (0.10) ** 2  # (m/s^2)^2

        self.err_count = 0
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"imu_driver up: bus={g('i2c_bus')} addr=0x{self.addr:02x} "
            f"rate={rate_hz:.0f}Hz gyro_bias=({self.gyro_bias[0]:+.3f},"
            f"{self.gyro_bias[1]:+.3f},{self.gyro_bias[2]:+.3f})deg/s "
            f"accel_scale={self.accel_scale:.3f}")

    # ------------------------------------------------------------------ #
    def _init_chip(self):
        who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if who not in (0x68, 0x69):
            self.get_logger().warn(f"unexpected WHO_AM_I=0x{who:02x} (clone?)")
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)  # wake, PLL gyro-X clock
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, REG_CONFIG, 0x02)      # DLPF 94/98 Hz, 1 kHz rate
        self.bus.write_byte_data(self.addr, REG_SMPLRT_DIV, 0x00)  # keep internal 1 kHz
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG, 0x00)   # +/-250 deg/s
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x00)  # +/-2 g
        time.sleep(0.05)

    def _read_raw(self):
        """-> (ax, ay, az) in g, (gx, gy, gz) in deg/s — uncorrected."""
        raw = self.bus.read_i2c_block_data(self.addr, REG_DATA, 14)
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", bytes(raw))
        return ((ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G),
                (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS))

    def _calibrate(self, n, auto_scale):
        self.get_logger().info(f"calibrating ({n} samples) — keep the robot still ...")
        acc_sum = [0.0, 0.0, 0.0]
        gyr_sum = [0.0, 0.0, 0.0]
        for _ in range(n):
            a, w = self._read_raw()
            for i in range(3):
                acc_sum[i] += a[i]
                gyr_sum[i] += w[i]
            time.sleep(0.005)
        self.gyro_bias = tuple(s / n for s in gyr_sum)
        g_mag = math.sqrt(sum((s / n) ** 2 for s in acc_sum))
        if auto_scale:
            if 0.2 < g_mag < 2.0:
                self.accel_scale = 1.0 / g_mag
            else:
                self.get_logger().error(
                    f"gravity magnitude {g_mag:.2f} g implausible — was the robot "
                    f"moving? keeping accel_scale=1.0")

    # ------------------------------------------------------------------ #
    def _tick(self):
        try:
            a, w = self._read_raw()
        except OSError as e:
            self.err_count += 1
            if self.err_count in (1, 10, 100) or self.err_count % 1000 == 0:
                self.get_logger().error(f"I2C read failed x{self.err_count}: {e}")
            return
        self.err_count = 0

        m = self.msg
        m.header.stamp = self.get_clock().now().to_msg()
        m.linear_acceleration.x = a[0] * self.accel_scale * G
        m.linear_acceleration.y = a[1] * self.accel_scale * G
        m.linear_acceleration.z = a[2] * self.accel_scale * G
        m.angular_velocity.x = math.radians(w[0] - self.gyro_bias[0])
        m.angular_velocity.y = math.radians(w[1] - self.gyro_bias[1])
        m.angular_velocity.z = math.radians(w[2] - self.gyro_bias[2])
        self.pub.publish(m)

    def shutdown(self):
        try:
            self.bus.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
