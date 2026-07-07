#!/usr/bin/env python3
"""
detection_bpu — YOLO11 object detection on the BPU -> /perception/detections.

Idle (no model loaded, no BPU load) until /perception/yolo11_enable latches
true — the model is lazy-loaded on first enable so a robot that never
toggles this on pays no startup cost for it. While enabled, the front
camera is inferred every tick; left/right alternate to bound BPU time
(mirrors scripts/08_yolo_avoid.py's timing: YOLO11m ~52 ms/frame on this
board, front + one side per cycle).

Detections are normalized 0..1 against each frame's width/height and
published with their COCO class name directly, so consumers never need a
synced class-id table.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from navbot_msgs.msg import Detections

from .imgproc import decode

ZOO = "/home/sunrise/rdk_model_zoo"
YOLO_DIR = ZOO + "/samples/vision/ultralytics_yolo/runtime/python"
YOLO_BIN = ZOO + "/samples/vision/ultralytics_yolo/model/yolo11m_detect_bayese_640x640_nv12.bin"
STALE_S = 0.6

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


def _load_yolo():
    """Import the model-zoo wrapper without depending on the caller's cwd
    (same loader as scripts/08_yolo_avoid.py)."""
    sys.path.insert(0, ZOO)              # for `utils.py_utils.*`
    sys.path.insert(0, YOLO_DIR)
    from ultralytics_yolo_det import (UltralyticsYOLODetect,
                                      UltralyticsYOLODetectConfig)
    return UltralyticsYOLODetect(
        UltralyticsYOLODetectConfig(model_path=YOLO_BIN, score_thres=0.35))


class DetectionBpu(Node):
    def __init__(self):
        super().__init__("detection_bpu")

        self.declare_parameter("rate_hz", 8.0)
        self.declare_parameter("front_topic", "/cam_front/image_raw")
        self.declare_parameter("left_topic", "/cam_left/image_raw")
        self.declare_parameter("right_topic", "/cam_right/image_raw")
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.enabled = False
        self.det = None                  # lazy-loaded on first enable
        self.cams = {c: dict(frame=None, stamp=0.0) for c in ("front", "left", "right")}
        self._side_toggle = 0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/perception/yolo11_enable",
                                 self._on_enable, latched)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, g("front_topic"),
                                 lambda m: self._store("front", m), qos)
        self.create_subscription(Image, g("left_topic"),
                                 lambda m: self._store("left", m), qos)
        self.create_subscription(Image, g("right_topic"),
                                 lambda m: self._store("right", m), qos)

        self.pub = self.create_publisher(Detections, "/perception/detections", 10)
        self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.get_logger().info("detection_bpu up (idle until enabled)")

    def _store(self, key, msg):
        self.cams[key].update(frame=msg, stamp=time.time())

    def _on_enable(self, msg):
        self.enabled = bool(msg.data)
        if self.enabled and self.det is None:
            self.get_logger().info("loading YOLO11 ...")
            self.det = _load_yolo()
            self.get_logger().info("YOLO11 loaded")

    def _infer_and_publish(self, name):
        c = self.cams[name]
        if c["frame"] is None or time.time() - c["stamp"] > STALE_S:
            return
        bgr = decode(c["frame"])
        if bgr is None:
            return
        boxes, scores, cls = self.det.predict(bgr)
        h, w = bgr.shape[:2]
        msg = Detections()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.camera = name
        msg.x1 = [float(b[0] / w) for b in boxes]
        msg.y1 = [float(b[1] / h) for b in boxes]
        msg.x2 = [float(b[2] / w) for b in boxes]
        msg.y2 = [float(b[3] / h) for b in boxes]
        msg.score = [float(s) for s in scores]
        msg.class_name = [COCO_NAMES[int(k)] if 0 <= int(k) < len(COCO_NAMES)
                          else f"class-{int(k)}" for k in cls]
        self.pub.publish(msg)

    def _tick(self):
        if not self.enabled or self.det is None:
            return
        self._infer_and_publish("front")
        side = ("left", "right")[self._side_toggle]
        self._side_toggle ^= 1
        self._infer_and_publish(side)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionBpu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
