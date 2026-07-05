#!/usr/bin/env python3
"""Step 12 - MPU6050 noise characterization (VIO SLAM plan, Phase 1.2).

Robot MUST be completely still (motors off, nobody touching the bench).
Reads the IMU directly over I2C (no ROS in the loop) for `--seconds`,
then reports per-axis:

  * gyro noise density  [rad/s/sqrt(Hz)]  and white-noise sigma [rad/s]
  * accel noise density [m/s^2/sqrt(Hz)]  and sigma [m/s^2]
  * achieved sample rate (sanity check for the 200 Hz driver target)

Values go into navbot_slam/imu_driver covariance parameters and (later)
the SLAM backend's IMU noise config. Clone-chip accel scale is corrected
the same way imu_driver does it (gravity-norm rescale).

Run:      sudo python3 scripts/12_imu_noise.py --seconds 120
"""
import argparse
import math
import struct
import time

import numpy as np
from smbus2 import SMBus

G = 9.80665
ADDR = 0x68


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--bus", type=int, default=5)
    args = ap.parse_args()

    bus = SMBus(args.bus)
    bus.write_byte_data(ADDR, 0x6B, 0x01)      # wake, PLL gyro-X clock
    time.sleep(0.05)
    bus.write_byte_data(ADDR, 0x1A, 0x02)      # DLPF ~94/98 Hz, 1 kHz rate
    bus.write_byte_data(ADDR, 0x19, 0x00)
    bus.write_byte_data(ADDR, 0x1B, 0x00)      # ±250 dps
    bus.write_byte_data(ADDR, 0x1C, 0x00)      # ±2 g
    time.sleep(0.05)

    print(f"sampling for {args.seconds:.0f} s — keep the robot perfectly still")
    acc, gyr, stamps = [], [], []
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        raw = bus.read_i2c_block_data(ADDR, 0x3B, 14)
        stamps.append(time.monotonic())
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", bytes(raw))
        acc.append((ax / 16384.0, ay / 16384.0, az / 16384.0))
        gyr.append((gx / 131.0, gy / 131.0, gz / 131.0))

    acc = np.asarray(acc)
    gyr = np.radians(np.asarray(gyr))          # rad/s
    dt = np.diff(stamps)
    rate = 1.0 / dt.mean()
    print(f"\n{len(acc)} samples, rate {rate:.1f} Hz "
          f"(dt jitter std {dt.std()*1e3:.2f} ms)")

    # clone-chip accel scale: rescale so |gravity| = 1 g (same as imu_driver)
    g_mag = np.linalg.norm(acc.mean(axis=0))
    scale = 1.0 / g_mag if 0.2 < g_mag < 2.0 else 1.0
    acc_ms2 = acc * scale * G
    print(f"accel scale factor (gravity-norm): {scale:.3f}")

    print(f"\n{'axis':<6}{'gyro sigma rad/s':<20}{'gyro dens rad/s/rtHz':<24}"
          f"{'accel sigma m/s^2':<20}{'accel dens m/s^2/rtHz'}")
    g_sig = gyr.std(axis=0)
    a_sig = acc_ms2.std(axis=0)
    rt = math.sqrt(rate / 2.0)                 # sigma = density * sqrt(BW), BW = rate/2
    for i, axn in enumerate("xyz"):
        print(f"{axn:<6}{g_sig[i]:<20.5f}{g_sig[i]/rt:<24.6f}"
              f"{a_sig[i]:<20.5f}{a_sig[i]/rt:.6f}")

    gv, av = float((g_sig ** 2).max()), float((a_sig ** 2).max())
    print(f"\nimu_driver covariance parameters (worst axis):")
    print(f"  gyro_variance:  {gv:.2e}   # (rad/s)^2")
    print(f"  accel_variance: {av:.2e}   # (m/s^2)^2")
    print(f"SLAM noise densities (worst axis): "
          f"gyr {g_sig.max()/rt:.2e} rad/s/rtHz, acc {a_sig.max()/rt:.2e} m/s^2/rtHz")


if __name__ == "__main__":
    main()
