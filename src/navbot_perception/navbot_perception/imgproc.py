"""Shared image helpers for the BPU perception nodes (obstacle_fusion,
detection_bpu, depth_bpu) — camera message decoding and the BGR->NV12
conversion the Horizon BPU wants as model input."""

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image


def bgr2nv12(bgr):
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(h * w * 3 // 2)
    y = yuv[: h * w]
    uv = yuv[h * w:].reshape(2, h * w // 4).transpose(1, 0).reshape(h * w // 2)
    return np.concatenate([y, uv])


def decode(msg):
    """sensor_msgs Image/CompressedImage -> BGR ndarray (None if unsupported)."""
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    buf = np.frombuffer(msg.data, np.uint8)
    enc = msg.encoding.lower()
    if enc in ("yuyv", "yuv422_yuy2", "yuy2"):
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 2),
                            cv2.COLOR_YUV2BGR_YUY2)
    if enc == "nv12":
        return cv2.cvtColor(buf.reshape(msg.height * 3 // 2, msg.width),
                            cv2.COLOR_YUV2BGR_NV12)
    if enc == "bgr8":
        return buf.reshape(msg.height, msg.width, 3)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3),
                            cv2.COLOR_RGB2BGR)
    return None
