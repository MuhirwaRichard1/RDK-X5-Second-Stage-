#!/usr/bin/env python3
"""Step 3 - TF-Luna over I2C.

Streams distance, signal strength and internal temperature at 20 Hz.

Run:      sudo python3 03_tfluna_test.py
Pass if:  distance tracks a hand or wall moved between ~0.2 m and 3 m,
          amp stays in a healthy range on normal targets, and no
          I/O errors appear over a full minute.

Reading quality rules (from the Benewake datasheet):
  amp < 100     -> signal too weak, distance not trustworthy
  amp == 65535  -> sensor saturated (target too close / too reflective)
This is exactly the gate your safety node should apply before trusting
a range for the depth-scale calibration or forward stop.
"""
import time

from smbus2 import SMBus

BUS, ADDR = 5, 0x10
FRAME_REG = 0x00   # DIST_L, DIST_H, AMP_L, AMP_H, TEMP_L, TEMP_H

with SMBus(BUS) as bus:
    print("streaming at 20 Hz - Ctrl+C to stop")
    try:
        while True:
            d = bus.read_i2c_block_data(ADDR, FRAME_REG, 6)
            dist_cm = d[0] | (d[1] << 8)
            amp = d[2] | (d[3] << 8)
            temp_c = (d[4] | (d[5] << 8)) / 100.0

            if amp < 100:
                quality = "WEAK - ignore reading"
            elif amp == 65535:
                quality = "SATURATED - ignore reading"
            else:
                quality = "ok"

            print(f"dist {dist_cm:4d} cm | amp {amp:5d} | "
                  f"{temp_c:5.1f} C | {quality:26s}", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\ndone")