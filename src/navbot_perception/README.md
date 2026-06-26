# navbot_perception
On-BPU perception + CPU obstacle fusion.
- `depth_bpu` — **Depth Anything** `.bin` on the **BPU** (primary). **In:** front image (NV12).
  **Out:** `/perception/depth` (sensor_msgs/Image, 32FC1 m or 16UC1 mm, ~5Hz). See
  [../../docs/depth_anything_conversion.md](../../docs/depth_anything_conversion.md).
- `detection_bpu` — YOLO11 `.bin` on the BPU (time-shared, secondary). **Out:** `/perception/detections`.
- `obstacle_fusion` — ground/obstacle split over 3 cams (+depth) → `/obstacles` (navbot_msgs/Sectors, 8Hz).
  Unknown sector = blocked (fail-safe).
