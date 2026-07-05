#!/usr/bin/env python3
"""
Test 6 -- Integration smoke test: drive forward, stop before an obstacle.
Combines everything: L298N drive, TF-Luna range gating, MPU6050 heading drift.

WHEELS ON THE GROUND for this one. Give the robot ~2 m of clear runway
toward a wall or a box.

Behaviour:
  1. Robot drives forward at DRIVE_DUTY.
  2. TF-Luna is polled at 20 Hz. If distance < STOP_CM (with good signal),
     motors stop immediately.
  3. While driving, gyro-Z is integrated to report how much the heading
     drifted (a perfectly matched drivetrain would read ~0 deg).
  4. Ctrl+C at any time = emergency stop.

Pass criteria:
  - Robot stops before touching the obstacle.
  - Reported heading drift is small (a few degrees over 1-2 m). Large drift
    means the motors are mismatched -- feed the stiction/LUT data from
    test 05 into config/drive_lut.yaml.

Run:  sudo python3 06_obstacle_stop_demo.py
"""
import time
import smbus2
import Hobot.GPIO as GPIO

# ---- I2C ----
BUS_ID   = 5
MPU_ADDR = 0x68
TFL_ADDR = 0x10

# ---- L298N pins (BOARD numbering) ----
ENA, ENB = 32, 33          # PWM3, default-enabled
IN1, IN2 = 16, 18          # left motor
IN3, IN4 = 22, 36          # right motor
PWM_HZ   = 1000

# ---- Behaviour tuning ----
DRIVE_DUTY = 40            # % duty while cruising (raise if robot stalls)
STOP_CM    = 30            # stop when TF-Luna reads closer than this
MIN_AMP    = 100           # ignore TF-Luna frames weaker than this
LOOP_HZ    = 20            # sensor poll rate
TIMEOUT_S  = 15            # give up after this long even with no obstacle

GYRO_Z_SCALE = 131.0       # LSB per deg/s at +/-250 dps


def mpu_init(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)   # wake up
    time.sleep(0.1)


def mpu_gyro_z(bus):
    hi, lo = bus.read_i2c_block_data(MPU_ADDR, 0x47, 2)
    raw = (hi << 8) | lo
    if raw > 32767:
        raw -= 65536
    return raw / GYRO_Z_SCALE                    # deg/s


def tfl_read(bus):
    d = bus.read_i2c_block_data(TFL_ADDR, 0x00, 6)
    dist = d[0] | (d[1] << 8)                    # cm
    amp  = d[2] | (d[3] << 8)
    return dist, amp


def main():
    bus = smbus2.SMBus(BUS_ID)
    mpu_init(bus)

    # brief gyro-Z bias calibration -- robot must be STILL
    print("calibrating gyro bias, keep the robot still ...")
    bias = sum(mpu_gyro_z(bus) for _ in range(100)) / 100.0
    print(f"  gyro-Z bias = {bias:+.2f} deg/s")

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([ENA, ENB], GPIO.OUT)
    pwm_a = GPIO.PWM(ENA, PWM_HZ)
    pwm_b = GPIO.PWM(ENB, PWM_HZ)
    pwm_a.start(0)
    pwm_b.start(0)

    def forward(duty):
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
        pwm_a.ChangeDutyCycle(duty)
        pwm_b.ChangeDutyCycle(duty)

    def stop():
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)
        GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

    heading = 0.0
    dt = 1.0 / LOOP_HZ
    reason = "timeout"

    print(f"driving forward at {DRIVE_DUTY}% -- will stop at {STOP_CM} cm")
    t0 = time.time()
    forward(DRIVE_DUTY)
    try:
        while time.time() - t0 < TIMEOUT_S:
            loop_start = time.time()

            dist, amp = tfl_read(bus)
            heading += (mpu_gyro_z(bus) - bias) * dt

            good = MIN_AMP <= amp and amp != 65535
            print(f"  dist {dist:4d} cm  amp {amp:5d}  "
                  f"heading {heading:+6.1f} deg", end="\r")

            if good and dist < STOP_CM:
                reason = f"obstacle at {dist} cm"
                break

            sleep = dt - (time.time() - loop_start)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        reason = "user e-stop (Ctrl+C)"
    finally:
        stop()
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        bus.close()

    print(f"\nSTOPPED: {reason}")
    print(f"heading drift while driving: {heading:+.1f} deg")
    if abs(heading) > 10:
        print("  -> large drift: motors are mismatched, use the stiction data"
              " from test 05 to build config/drive_lut.yaml")
    else:
        print("  -> drivetrain looks reasonably matched, PASS")


if __name__ == "__main__":
    main()
