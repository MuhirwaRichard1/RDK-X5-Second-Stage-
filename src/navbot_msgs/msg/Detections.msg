# YOLO11 detection boxes for one camera frame. Carries class names directly
# (rather than numeric class ids) so consumers never need a synced COCO
# id -> name table.
#
# Arrays are parallel and the same length (one entry per detected box).
# Box coordinates are normalized 0..1 against the source frame's width/height.

std_msgs/Header header
string camera             # "front" | "left" | "right"
float32[] x1
float32[] y1
float32[] x2
float32[] y2
float32[] score
string[] class_name
