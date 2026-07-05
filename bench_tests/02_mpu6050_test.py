#!/usr/bin/env python3
"""Step 2 - MPU6050 bring-up.

Wakes the IMU from sleep and streams accel/gyro at 10 Hz with a
simple accelerometer-only pitch/roll estimate.

Run:      sudo python3 02_mpu6050_test.py
Pass if:  flat on the bench az is about +1.00 g, gyro is about 0
          (within +/-3 deg/s of bias), and tilting the board moves
          pitch/roll smoothly with no freezes or I/O errors.

This is the same data your /imu/data ROS 2 node will publish, so if
the numbers are stable here, the VIO pipeline gets clean input.
"""
import math
import sys
import time

from smbus2 import SMBus

BUS, ADDR = 5, 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
WHO_AM_I = 0x75

ACCEL_LSB_PER_G = 16384.0   # +/-2 g full scale (default)
GYRO_LSB_PER_DPS = 131.0    # +/-250 deg/s full scale (default)


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


with SMBus(BUS) as bus:
    if bus.read_byte_data(ADDR, WHO_AM_I) != 0x68:
        sys.exit("WHO_AM_I mismatch - wrong device on 0x68?")

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)   # clear sleep bit
    time.sleep(0.1)
    print("streaming at 10 Hz - Ctrl+C to stop")
    print("   ax     ay     az   |    gx      gy      gz    |  pitch   roll")
    print("  (g)    (g)    (g)   |  (deg/s) (deg/s) (deg/s) |  (deg)   (deg)")

    try:
        while True:
            d = bus.read_i2c_block_data(ADDR, ACCEL_XOUT_H, 14)
            ax = s16(d[0], d[1]) / ACCEL_LSB_PER_G
            ay = s16(d[2], d[3]) / ACCEL_LSB_PER_G
            az = s16(d[4], d[5]) / ACCEL_LSB_PER_G
            gx = s16(d[8], d[9]) / GYRO_LSB_PER_DPS
            gy = s16(d[10], d[11]) / GYRO_LSB_PER_DPS
            gz = s16(d[12], d[13]) / GYRO_LSB_PER_DPS

            pitch = math.degrees(math.atan2(ax, math.hypot(ay, az)))
            roll = math.degrees(math.atan2(ay, math.hypot(ax, az)))

            print(f" {ax:+5.2f}  {ay:+5.2f}  {az:+5.2f}  | "
                  f"{gx:+7.1f} {gy:+7.1f} {gz:+7.1f}  | "
                  f"{pitch:+6.1f}  {roll:+6.1f}   ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\ndone")
