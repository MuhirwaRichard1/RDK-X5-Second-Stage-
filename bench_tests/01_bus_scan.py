#!/usr/bin/env python3
"""Step 1 - I2C bus sanity check.

Verifies that both sensors answer on I2C5 (physical pins 3/5).

Run:      sudo python3 01_bus_scan.py
Pass if:  MPU6050 responds at 0x68, TF-Luna responds at 0x10,
          and WHO_AM_I reads back 0x68.

If a device is missing, first check with:  i2cdetect -y -r 5
- Nothing at 0x68 -> MPU6050 VCC/SDA/SCL swapped or loose.
- Nothing at 0x10 -> TF-Luna pin 5 (config) is probably NOT tied to GND,
  so it booted in UART mode. Power-cycle it after grounding pin 5.
"""
import sys

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 missing - run: sudo pip3 install smbus2")

BUS = 5
DEVICES = {0x68: "MPU6050 (IMU)", 0x10: "TF-Luna (LiDAR)"}
WHO_AM_I = 0x75

ok = True
with SMBus(BUS) as bus:
    for addr, name in DEVICES.items():
        try:
            bus.read_byte(addr)
            print(f"[ OK ] {name:16s} responding at 0x{addr:02X} on i2c-{BUS}")
        except OSError:
            ok = False
            print(f"[FAIL] {name:16s} not found at 0x{addr:02X}")

    if ok:
        who = bus.read_byte_data(0x68, WHO_AM_I)
        status = "OK" if who == 0x68 else "UNEXPECTED"
        print(f"[{status:^4s}] MPU6050 WHO_AM_I = 0x{who:02X} (expected 0x68)")

sys.exit(0 if ok else 1)
