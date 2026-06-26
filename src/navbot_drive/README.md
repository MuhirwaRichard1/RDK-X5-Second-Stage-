# navbot_drive
- `safety_gate` — clamps `/cmd_vel`→`/cmd_vel_safe` on E-stop / TF-Luna proximity / input timeout. Isolated RT core.
- `motor_controller` — Twist→duty via `config/drive_lut.yaml`, drives L298N ENA/ENB (PWM) + IN1–4 (GPIO). Dead-man stop on 200ms timeout.
