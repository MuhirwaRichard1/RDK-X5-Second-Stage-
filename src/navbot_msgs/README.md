# navbot_msgs
Custom interfaces.
- `Sectors.msg` — fused per-direction obstacle proximity (output of obstacle_fusion).
- `Calibrate.action` — duty-cycle ↔ velocity calibration routine for the encoderless drive.
- `MapIO.srv` — save/load the SLAM map to/from disk (`/save_map`, `/load_map`).
