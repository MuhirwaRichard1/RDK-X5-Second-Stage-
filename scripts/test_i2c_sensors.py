#!/usr/bin/env python3
"""
test_i2c_sensors — bench check for the NavBot's I2C5 sensors (header pins 3=SDA, 5=SCL).

  MPU6050  IMU        addr 0x68 (0x69 if AD0 is pulled high)
  TF-Luna  lidar      addr 0x10 (its pin 5/config must be tied to GND for I2C mode,
                                 otherwise it boots in UART mode and never ACKs)

Usage:  python3 scripts/test_i2c_sensors.py [--bus 5] [--seconds 5]

Prints a bus scan first, then streams readings from whichever sensors respond.
"""

import argparse
import struct
import time

from smbus2 import SMBus, i2c_msg

MPU_ADDRS = (0x68, 0x69)
TFLUNA_ADDR = 0x10


def scan(bus):
    # read_byte, not write_quick: the X5's DesignWare adapter doesn't do
    # SMBus Quick, so write_quick raises OSError for every address.
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    return found


# --------------------------- MPU6050 --------------------------- #
def mpu_init(bus, addr):
    who = bus.read_byte_data(addr, 0x75)          # WHO_AM_I
    bus.write_byte_data(addr, 0x6B, 0x00)         # PWR_MGMT_1: wake up
    bus.write_byte_data(addr, 0x1B, 0x00)         # GYRO_CONFIG: ±250 °/s
    bus.write_byte_data(addr, 0x1C, 0x00)         # ACCEL_CONFIG: ±2 g
    time.sleep(0.05)
    return who


def mpu_read(bus, addr):
    raw = bus.read_i2c_block_data(addr, 0x3B, 14)  # accel, temp, gyro
    ax, ay, az, t, gx, gy, gz = struct.unpack(">7h", bytes(raw))
    return (ax / 16384.0, ay / 16384.0, az / 16384.0,   # g  (±2g default)
            t / 340.0 + 36.53,                          # °C
            gx / 131.0, gy / 131.0, gz / 131.0)         # °/s (±250 default)


# --------------------------- TF-Luna --------------------------- #
def tfluna_read(bus, addr):
    # regs 0x00..0x05: dist_lo/hi (cm), amp_lo/hi, temp_lo/hi (0.01 °C)
    w = i2c_msg.write(addr, [0x00])
    r = i2c_msg.read(addr, 6)
    bus.i2c_rdwr(w, r)
    d = list(r)
    dist = d[0] | d[1] << 8
    amp = d[2] | d[3] << 8
    temp = (d[4] | d[5] << 8) / 100.0
    return dist, amp, temp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=5)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    with SMBus(args.bus) as bus:
        found = scan(bus)
        print(f"i2c-{args.bus} scan: "
              + (", ".join(f"0x{a:02x}" for a in found) if found else "NO DEVICES"))

        mpu = next((a for a in MPU_ADDRS if a in found), None)
        luna = TFLUNA_ADDR if TFLUNA_ADDR in found else None

        if not found:
            print("\nNothing ACKed. Check: SDA->pin3, SCL->pin5, common GND, sensor VCC,")
            print("and TF-Luna pin 5 tied to GND (I2C mode select).")
            return 1
        if mpu:
            who = mpu_init(bus, mpu)
            print(f"MPU6050 @0x{mpu:02x}: WHO_AM_I=0x{who:02x} "
                  f"({'OK' if who in MPU_ADDRS else 'UNEXPECTED'})")
        else:
            print("MPU6050 missing (expected 0x68/0x69)")
        if luna:
            print(f"TF-Luna @0x{luna:02x}: OK")
        else:
            print("TF-Luna missing (expected 0x10) — is its config pin (pin 5) on GND?")

        t_end = time.time() + args.seconds
        while time.time() < t_end and (mpu or luna):
            line = []
            if mpu:
                ax, ay, az, t, gx, gy, gz = mpu_read(bus, mpu)
                line.append(f"acc=({ax:+.2f},{ay:+.2f},{az:+.2f})g "
                            f"gyro=({gx:+6.1f},{gy:+6.1f},{gz:+6.1f})°/s {t:.1f}°C")
            if luna:
                dist, amp, temp = tfluna_read(bus, luna)
                ok = "weak-signal!" if amp < 100 else ""
                line.append(f"lidar={dist}cm amp={amp} {ok}")
            print(" | ".join(line))
            time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
