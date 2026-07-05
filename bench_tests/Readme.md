# NavBot Bench Tests -- RDK X5

Progressive bring-up tests for the Tri-Cam NavBot base hardware:
MPU6050 + TF-Luna (I2C mode) on I2C5, two brushed motors via L298N.

## Wiring recap (BOARD pin numbers)

| RDK X5 pin | Signal      | Goes to                                  |
|-----------|-------------|-------------------------------------------|
| 1         | 3.3V        | MPU6050 VCC                                |
| 2         | 5V          | TF-Luna pin 1 (VCC)                        |
| 3         | I2C5 SDA    | MPU6050 SDA + TF-Luna pin 2 (SDA)          |
| 5         | I2C5 SCL    | MPU6050 SCL + TF-Luna pin 3 (SCL)          |
| 6         | GND         | MPU6050 GND + AD0 (locks addr 0x68)        |
| 9         | GND         | TF-Luna pin 4 (GND) + pin 5 (selects I2C)  |
| 16        | GPIO        | L298N IN1                                  |
| 18        | GPIO        | L298N IN2                                  |
| 22        | GPIO        | L298N IN3                                  |
| 36        | GPIO        | L298N IN4                                  |
| 32        | PWM3        | L298N ENA (jumper cap removed)             |
| 33        | PWM3        | L298N ENB (jumper cap removed)             |
| 39        | GND         | L298N GND (common ground with battery)     |

Power: battery -> L298N VS and battery -> UBEC 5V/5A -> RDK X5 USB-C.
Common ground between battery, L298N, UBEC, and the RDK X5 is mandatory.

## Prerequisites

```bash
sudo pip3 install smbus2      # Hobot.GPIO ships with the RDK image
```

Pins 32/33 are on PWM3, which is enabled by default -- no srpi-config needed.

## Run order

| # | File                        | Robot state   | Verifies                          |
|---|-----------------------------|---------------|-----------------------------------|
| 1 | 01_bus_scan.py              | powered, still| both devices ACK on i2c-5, WHO_AM_I |
| 2 | 02_mpu6050_test.py          | flat, still   | accel ~ +1.00 g on Z, gyro near 0 |
| 3 | 03_tfluna_test.py           | aimed at wall | sane distance, amp > 100          |
| 4 | 04_motor_direction_test.py  | WHEELS UP     | forward = forward on both sides   |
| 5 | 05_drive_functions.py       | WHEELS UP     | PWM ramp + stiction start duty    |
| 6 | 06_obstacle_stop_demo.py    | on ground     | drive + range stop + drift check  |

All scripts: `sudo python3 <file>`. Ctrl+C is always a safe stop.

## Safety

- Motor tests 04/05: robot on a stand, wheels off the ground.
- Never hot-plug anything on the 40-pin header -- power down first.
- If TF-Luna doesn't appear at 0x10, its pin 5 isn't grounded and it booted
  into UART mode. Ground pin 5 and power-cycle.
