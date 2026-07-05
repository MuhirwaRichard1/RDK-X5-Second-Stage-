# Bill of Materials — RDK X5 Tri-Cam NavBot

> **Version:** 1.0 &nbsp;|&nbsp; **Date:** 2026-06-26 &nbsp;|&nbsp; Currency: USD (estimates, for planning only)


| # | Part | Qty | Interface / Voltage | Supplier / SKU | Est. unit | Notes |
|---|---|---|---|---|---|---|
| 1 | RDK X5 (8×A55, ~10 TOPS BPU) | 1 | 5V / 3A | D-Robotics | $160 | The brain; runs ROS 2 Humble + TROS |
| 2 | HBVCAM OV2710 100° (front) | 1 | USB-2 UVC, 5V | HBVCAM | ~$25 | MJPEG 1280×720; primary nav + detection cam |
| 3 | Wide-angle USB camera (L/R) | 2 | USB-2 UVC, 5V | generic ELP/wide-FOV | ~$36 | Side surround; 640×480 MJPEG is enough |
| 4 | **4-port USB 3.0 hub / extender** | 1 | USB 3.0 (to 1 X5 port) | generic | ~$10 | All 3 cams share **one** X5 USB port via this hub; powered/extender variant for reach (Risk R2) |
| 5 | DC geared motor (no encoder) | 2 | 6–12V | generic TT/gearmotor | ~$5 | Differential drive; calibrated via duty↔vel LUT |
| 6 | L298N dual H-bridge | 1 | motor 6–12V; logic 5V; ENA/ENB PWM, IN1–4 GPIO | generic | ~$4 | Drives both motors; ENA/ENB = PWM speed |
| 7 | TF-Luna LiDAR (range) | 1 | I2C (3.3–5V) | Benewake | $30 | Forward safety range; already wired on I2C5 @ 0x10 |
| 8 | **IMU — MPU6050** *(confirmed)* | 1 | I2C (3.3V) @ 0x68 | InvenSense | ~$6 | 6-axis; fuses with cameras for **VIO + SLAM**; essential for kidnap recovery (Risk R1) |
| 9 | LiPo / 18650 battery pack | 1 | 7.4–11.1V | generic | ~$15 | ≥ 20 min runtime target |
| 10 | 5V/5A UBEC / buck regulator | 1 | Vbat→5V | generic | ~$6 | Clean 5V for board+cams; **separate motor rail** |
| 11 | Chassis + 2 wheels + caster | 1 | — | generic robot chassis | ~$15 | Differential platform |
| 12 | Jumper wires / resistors / LEDs | — | — | generic | ~$5 | Status LEDs already used in bench scripts |
| | **Estimated build total (excl. RDK X5 & TF-Luna already owned)** | | | | **≈ $316** | Compute + sensing under target $150 |

## Power architecture (important for encoderless reliability)
```
Battery (7.4-11.1V)
   ├──► L298N motor supply (VS)         ── DC motors (noisy rail, kept separate)
   └──► UBEC/buck 5V/5A
            ├──► RDK X5 5V/3A           (clean logic rail)
            ├──► Powered USB hub        ── 3 USB cameras
            └──► L298N logic 5V (VSS)   (common GND with board — REQUIRED)
```

## Interface budget
| Bus | Devices | Concern |
|---|---|---|
| **1× USB 3.0 port → 4-port hub** | 3 cameras | The cameras are UVC **USB 2.0** devices, so even through a USB 3.0 hub they share the host port's single **480 Mbps High-Speed** bus. MJPEG mandatory; sides at 640×480@15 to fit (Risk R2). The USB 3.0 hub mainly buys reach + clean power, not raw bandwidth for these cameras. |
| I2C5 | TF-Luna (0x10) + **MPU6050 IMU (0x68)** | distinct addresses, no conflict; confirm both with `i2cdetect -y -r 5` |
| GPIO + PWM | L298N IN1–4 + ENA/ENB | ENA=pin18 (pwm1, known-good); ENB=pin37 — verify a pwmchip exists; IN1–4 = plain GPIO. See `RDK_X5_Peripherals.md` for enabling extra PWM. |
