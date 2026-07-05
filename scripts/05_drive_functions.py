#!/usr/bin/env python3
"""Step 5 - drive primitives + stiction measurement.

Builds the differential-drive functions your motor_controller node
will wrap, then runs two experiments:

  1. PWM ramp 0 -> 100 -> 0 on both wheels (smoothness check).
  2. Stiction sweep: duty rises in 5% steps with pauses so you can
     note the minimum duty at which each wheel starts turning.
     That value is the floor of your duty<->velocity LUT
     (config/drive_lut.yaml) - below it, commanded motion is a lie
     the SLAM correction loop has to absorb.

Run:      sudo python3 05_drive_functions.py     (wheels up for ramp,
          then repeat on the floor per surface for the LUT)
"""
import time

import Hobot.GPIO as GPIO

ENA, ENB = 32, 33
IN1, IN2, IN3, IN4 = 16, 18, 22, 36
PWM_HZ = 1000


class Drive:
    """Signed duty in [-100, 100] per wheel: + forward, - reverse."""

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT, initial=GPIO.LOW)
        self.pwm_a = GPIO.PWM(ENA, PWM_HZ)
        self.pwm_b = GPIO.PWM(ENB, PWM_HZ)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        # Hobot.GPIO quirk: start(0)/ChangeDutyCycle never write sysfs
        # 'enable', so force it once (same workaround as motor_controller.py).
        for pin in (ENA, ENB):
            with open(GPIO.pin_info[pin].pwm_enable, "w") as f:
                f.write("1")

    def set(self, left, right):
        left = max(-100, min(100, left))
        right = max(-100, min(100, right))
        GPIO.output(IN1, GPIO.HIGH if left >= 0 else GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW if left >= 0 else GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH if right >= 0 else GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW if right >= 0 else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(abs(left))
        self.pwm_b.ChangeDutyCycle(abs(right))

    def forward(self, duty):   self.set(duty, duty)
    def backward(self, duty):  self.set(-duty, -duty)
    def spin_left(self, duty): self.set(-duty, duty)
    def spin_right(self, duty): self.set(duty, -duty)
    def stop(self):            self.set(0, 0)

    def close(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()


def ramp(drive):
    print("ramp: 0 -> 100 -> 0 in 5% steps")
    for duty in list(range(0, 101, 5)) + list(range(100, -1, -5)):
        drive.forward(duty)
        print(f"  duty {duty:3d}%", end="\r")
        time.sleep(0.25)
    drive.stop()
    print("\nramp done")


def stiction_sweep(drive):
    print("stiction sweep: watch for the duty where each wheel STARTS moving")
    for duty in range(0, 61, 5):
        drive.forward(duty)
        print(f"  holding duty {duty:3d}% for 2 s ...")
        time.sleep(2)
        drive.stop()
        time.sleep(0.5)
    print("record the start duty per wheel/surface in config/drive_lut.yaml")


if __name__ == "__main__":
    d = Drive()
    try:
        ramp(d)
        time.sleep(1)
        stiction_sweep(d)
    except KeyboardInterrupt:
        pass
    finally:
        d.close()
        print("motors released")