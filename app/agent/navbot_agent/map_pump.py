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
import os
import time

import cv2
import numpy as np
import yaml

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


def _render_saved(base):
    """Render a map saved on disk (<base>.pgm + <base>.yaml) the same way
    _render draws a live grid, so the console can show the chosen map the
    instant NAVIGATE starts instead of waiting for slam_toolbox to relocalize
    and publish /map. map_saver already writes the image top-row = +y, which
    is the flipped orientation the console expects — no flipud here."""
    path = os.path.join(config.MAP_DIR, base)
    with open(path + ".yaml") as f:
        meta = yaml.safe_load(f)
    # image: may be a bare name or a path — always resolve next to the yaml
    img_path = os.path.join(config.MAP_DIR,
                            os.path.basename(meta.get("image", base + ".pgm")))
    gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise OSError(f"unreadable map image: {img_path}")
    ok, buf = cv2.imencode(".png", gray)
    if not ok:
        raise OSError(f"png encode failed: {img_path}")
    origin = meta.get("origin") or (0.0, 0.0, 0.0)
    return (buf.tobytes(), gray.shape[1], gray.shape[0],
            float(meta.get("resolution", 0.0)),
            float(origin[0]), float(origin[1]))


class MapPump:
    def __init__(self, bridge):
        self.bridge = bridge
        self.hub = None
        self._preload = None            # map basename to show immediately

    def attach(self, hub):
        self.hub = hub

    def preload(self, base):
        """Show the saved map <base> right away (called when navigate starts).
        The live /map supersedes it as soon as slam_toolbox publishes one."""
        self._preload = base

    async def _send_preload(self, loop, last_seq):
        """Push the queued saved map once. Returns the seq the pump should
        treat as already sent — the grid left over from the previous mode must
        not overwrite the preview, only a /map published after it."""
        base, self._preload = self._preload, None
        try:
            png, w, h, res, ox, oy = await loop.run_in_executor(
                None, _render_saved, base)
        except Exception:                                         # noqa: BLE001
            log.exception("saved map preload failed: %s", base)
            return last_seq
        slot = self.bridge.map_slot
        seq = slot[6] if slot else last_seq
        self.hub.send_map(protocol.map_msg(
            seq, w, h, base64.b64encode(png).decode("ascii"),
            resolution=res, origin_x=ox, origin_y=oy))
        log.info("preloaded saved map %s (%dx%d)", base, w, h)
        return seq

    async def pump(self):
        loop = asyncio.get_running_loop()
        last_seq = -1
        while True:
            await asyncio.sleep(1.0 / config.MAP_PUSH_HZ)
            if not self.hub or not self.hub.wants_map():
                continue
            if self._preload is not None:
                last_seq = await self._send_preload(loop, last_seq)
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
            # slot = (data, width, height, resolution, origin_x, origin_y, seq, ..)
            self.hub.send_map(protocol.map_msg(
                seq, slot[1], slot[2], b64,
                resolution=slot[3], origin_x=slot[4], origin_y=slot[5]))
