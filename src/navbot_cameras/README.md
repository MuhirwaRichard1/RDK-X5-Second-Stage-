# navbot_cameras
Configures 3× USB cameras (wraps TROS `hobot_usb_cam`).
**Out:** `/cam_front/image_raw` (30Hz), `/cam_left/image_raw`, `/cam_right/image_raw` (15Hz) + camera_info.
**Failure mode:** device drop → auto-reopen, publish stale flag.
