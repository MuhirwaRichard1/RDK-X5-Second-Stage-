# navbot_slam
Visual-inertial SLAM, mapping, kidnap relocalization, and the IMU driver.
- `imu_driver` — MPU6050 on I2C5 @ 0x68 → `/imu/data` (sensor_msgs/Imu, 100Hz).
- `vio_slam` — fuse front-cam features + IMU + `/perception/depth` → `/odom` + `/tf` + `/map`;
  build/save/load map (`/save_map`, `/load_map`). Tracking lost → signals RELOCALIZE.
- `relocalizer` — match live view to the saved map → `/relocalized_pose` (kidnap recovery).
Candidate back-ends: ORB-SLAM3 / VINS-Fusion style, or RTAB-Map (RGB-D using Depth Anything depth).
