"""VideoPump — one asyncio task per camera, forwarding the bridge's
latest-wins frame slots to subscribed clients as binary WS messages.

Front camera arrives as JPEG (CompressedImage): "hd" = passthrough
(zero CPU), "sd" (default) = decode/resize/re-encode 640x360 q70 —
~3 Mbps instead of up to ~14, kinder to WiFi. Side cameras arrive as
raw YUYV 320x240 and are always JPEG-encoded here. cv2 work runs in
the default thread pool (cv2 releases the GIL)."""

import asyncio
import logging

import cv2
import numpy as np

from . import config

log = logging.getLogger("navbot.video")

_ENC = [int(cv2.IMWRITE_JPEG_QUALITY), config.VIDEO_JPEG_QUALITY]


def _encode_front_sd(jpeg_bytes):
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    img = cv2.resize(img, config.VIDEO_SD_SIZE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, _ENC)
    return buf.tobytes() if ok else None


def _encode_yuyv(raw, w, h):
    if len(raw) != w * h * 2:
        return None
    yuyv = np.frombuffer(raw, np.uint8).reshape(h, w, 2)
    bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
    ok, buf = cv2.imencode(".jpg", bgr, _ENC)
    return buf.tobytes() if ok else None


class VideoPump:
    def __init__(self, bridge):
        self.bridge = bridge
        self.hub = None
        self.settings = {cam: {"fps": config.VIDEO_DEFAULT_FPS[cam],
                               "quality": "sd"} for cam in config.CAMERAS}

    def attach(self, hub):
        self.hub = hub

    def configure(self, cam, fps=None, quality=None):
        s = self.settings[cam]
        if fps:
            s["fps"] = max(1.0, min(30.0, float(fps)))
        if quality in ("sd", "hd"):
            s["quality"] = quality

    async def pump(self, cam):
        cam_id = config.CAMERAS[cam]
        loop = asyncio.get_running_loop()
        last_seq = -1
        while True:
            await asyncio.sleep(1.0 / self.settings[cam]["fps"])
            if not self.hub or not self.hub.wants_video(cam):
                continue
            slot = self.bridge.frame_slots[cam]
            if slot is None:
                continue
            try:
                if slot[0] == "jpeg":
                    _, data, seq, mono = slot
                    if seq == last_seq:
                        continue
                    if self.settings[cam]["quality"] == "hd":
                        jpeg = data
                    else:
                        jpeg = await loop.run_in_executor(
                            None, _encode_front_sd, data)
                else:                               # ("yuyv", raw, w, h, seq, mono)
                    _, raw, w, h, seq, mono = slot
                    if seq == last_seq:
                        continue
                    jpeg = await loop.run_in_executor(
                        None, _encode_yuyv, raw, w, h)
            except Exception:                                   # noqa: BLE001
                log.exception("frame encode failed (%s)", cam)
                continue
            if jpeg is None:
                continue
            last_seq = seq
            self.hub.broadcast_video(cam, cam_id, seq, mono, jpeg)
