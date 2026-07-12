"""MapPump — one low-rate asyncio task forwarding the bridge's latest SLAM
occupancy grid to any client that has opted into the console's MAP view.

Unlike camera video, the map is a standalone picture (not composited onto
an independently-resizing live feed), so it's simplest to do all the
rendering here: decode the occupancy grid to a small grayscale PNG (free =
white, occupied = black, unknown = gray), bake the robot's position marker
directly into it when a fresh map->base_link TF is available (bridge.robot_pose),
base64-encode, and ship one flat JSON message — the desktop just decodes
and displays. cv2 work runs in the default thread pool, same as video.py."""

import asyncio
import base64
import logging
import time

import cv2
import numpy as np

from . import config, protocol

log = logging.getLogger("navbot.map")

_MARKER_COLOR = (0, 0, 255)      # BGR red
_MARKER_RADIUS = 4
_HEADING_LEN = 10
_POSE_STALE_S = 1.5


def _render(map_slot, robot_pose):
    data, width, height, resolution, origin_x, origin_y, seq, _mono = map_slot
    if width <= 0 or height <= 0:
        return None, seq
    arr = np.frombuffer(data, dtype=np.int8).reshape(height, width)
    gray = np.where(arr < 0, 128,
                    np.clip(255 - arr.astype(np.float32) * 2.55, 0, 255)
                    ).astype(np.uint8)
    gray = np.flipud(gray)          # image-up = world +y

    if robot_pose is None:
        ok, buf = cv2.imencode(".png", gray)
        return (buf.tobytes() if ok else None), seq

    x, y, yaw, _t = robot_pose
    col = (x - origin_x) / resolution
    row = height - 1 - (y - origin_y) / resolution
    if not (0 <= col < width and 0 <= row < height):
        ok, buf = cv2.imencode(".png", gray)
        return (buf.tobytes() if ok else None), seq

    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    px, py = int(col), int(row)
    end = (int(col + _HEADING_LEN * np.cos(yaw)),
          int(row - _HEADING_LEN * np.sin(yaw)))
    cv2.circle(bgr, (px, py), _MARKER_RADIUS, _MARKER_COLOR, -1)
    cv2.line(bgr, (px, py), end, _MARKER_COLOR, 2)
    ok, buf = cv2.imencode(".png", bgr)
    return (buf.tobytes() if ok else None), seq


class MapPump:
    def __init__(self, bridge):
        self.bridge = bridge
        self.hub = None

    def attach(self, hub):
        self.hub = hub

    async def pump(self):
        loop = asyncio.get_running_loop()
        last_seq = -1
        while True:
            await asyncio.sleep(1.0 / config.MAP_PUSH_HZ)
            if not self.hub or not self.hub.wants_map():
                continue
            slot = self.bridge.map_slot
            if slot is None or slot[6] == last_seq:
                continue
            pose = self.bridge.robot_pose
            if pose is not None and time.monotonic() - pose[3] > _POSE_STALE_S:
                pose = None
            try:
                png, seq = await loop.run_in_executor(None, _render, slot, pose)
            except Exception:                                     # noqa: BLE001
                log.exception("map render failed")
                continue
            if png is None:
                continue
            last_seq = seq
            b64 = base64.b64encode(png).decode("ascii")
            self.hub.send_map(protocol.map_msg(seq, slot[1], slot[2], b64))
