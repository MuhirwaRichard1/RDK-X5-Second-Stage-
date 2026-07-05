#!/usr/bin/env python3
"""Step 4 - L298N direction check.  *** WHEELS OFF THE GROUND ***

Runs each motor forward then reverse for 2 s at 50% duty so you can
verify wiring polarity before the robot ever touches the floor.

Pins (BOARD numbering):
  ENA = 32, ENB = 33   -> PWM3 group, enabled by default on RDK X5
  IN1 = 16, IN2 = 18   -> left motor direction
  IN3 = 22, IN4 = 36   -> right motor direction

Run:      sudo python3 04_motor_direction_test.py
Pass if:  "forward" spins each wheel in the robot's forward direction.
Fix:      if a motor spins the wrong way, swap its two wires on the
          L298N OUT terminals (hardware fix beats sign-flips in code).
"""
import time

import Hobot.GPIO as GPIO

ENA, ENB = 32, 33
IN1, IN2, IN3, IN4 = 16, 18, 22, 36
PWM_HZ = 1000
DUTY = 50

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT, initial=GPIO.LOW)
pwm_a = GPIO.PWM(ENA, PWM_HZ)
pwm_b = GPIO.PWM(ENB, PWM_HZ)
pwm_a.start(0)
pwm_b.start(0)


def stop_all():
    for pin in (IN1, IN2, IN3, IN4):
        GPIO.output(pin, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)


def run(step, in_hi, in_lo, pwm):
    print(step)
    GPIO.output(in_hi, GPIO.HIGH)
    GPIO.output(in_lo, GPIO.LOW)
    pwm.ChangeDutyCycle(DUTY)
    time.sleep(2)
    stop_all()
    time.sleep(1)


try:
    run("LEFT  motor FORWARD  (IN1=1 IN2=0)", IN1, IN2, pwm_a)
    run("LEFT  motor REVERSE  (IN1=0 IN2=1)", IN2, IN1, pwm_a)
    run("RIGHT motor FORWARD  (IN3=1 IN4=0)", IN3, IN4, pwm_b)
    run("RIGHT motor REVERSE  (IN3=0 IN4=1)", IN4, IN3, pwm_b)
    print("done - if any 'forward' was backwards, swap that motor's OUT wires")
finally:
    stop_all()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
