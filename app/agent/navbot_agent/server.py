"""WebSocket server: one connection per operator client.

Backpressure design: each session has ONE bounded outbox drained by ONE
sender task. Text messages drop-oldest when the outbox is full. Video
frames never queue — they land in a per-session latest-wins slot per
camera, with a single KICK sentinel in the outbox; a slow client simply
has its slots overwritten and memory stays bounded at one frame/camera."""

import asyncio
import json
import logging
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from . import protocol

log = logging.getLogger("navbot.server")

_VIDEO_KICK = object()


class ClientSession:
    def __init__(self, ws, name="client"):
        self.ws = ws
        self.name = name
        self.outbox = asyncio.Queue(maxsize=256)
        self.video_slots = {}          # cam_id -> packed frame (latest wins)
        self.video_cams = set()        # camera names this client wants
        self._kick_pending = False

    def send_text(self, text):
        try:
            self.outbox.put_nowait(text)
        except asyncio.QueueFull:
            try:
                self.outbox.get_nowait()               # drop oldest
                self.outbox.put_nowait(text)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass

    def send_json(self, msg):
        self.send_text(json.dumps(msg))

    def send_video(self, cam_id, payload):
        self.video_slots[cam_id] = payload
        if not self._kick_pending:
            self._kick_pending = True
            try:
                self.outbox.put_nowait(_VIDEO_KICK)
            except asyncio.QueueFull:
                self._kick_pending = False             # next frame re-kicks

    async def sender(self):
        while True:
            item = await self.outbox.get()
            if item is _VIDEO_KICK:
                self._kick_pending = False
                slots, self.video_slots = self.video_slots, {}
                for payload in slots.values():
                    await self.ws.send(payload)
            else:
                await self.ws.send(item)


class Hub:
    """Registry of live sessions + broadcast fan-out (asyncio thread only)."""

    def __init__(self):
        self.sessions = set()

    def broadcast(self, msg):
        if self.sessions:
            text = json.dumps(msg)
            for s in list(self.sessions):
                s.send_text(text)

    def broadcast_video(self, cam_name, cam_id, payload):
        for s in list(self.sessions):
            if cam_name in s.video_cams:
                s.send_video(cam_id, payload)

    def wants_video(self, cam_name):
        return any(cam_name in s.video_cams for s in self.sessions)


class WsServer:
    def __init__(self, app, host, port):
        self.app = app
        self.host = host
        self.port = port

    async def run(self):
        async with serve(self._handler, self.host, self.port,
                         max_size=4 * 1024 * 1024, compression=None):
            log.info("listening on ws://%s:%d", self.host, self.port)
            await asyncio.Future()                     # run forever

    async def _handler(self, ws):
        peer = getattr(ws, "remote_address", None)
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
        except (asyncio.TimeoutError, ConnectionClosed):
            return
        msg, err = protocol.parse_client(raw)
        if err or msg["type"] != "hello":
            await ws.send(json.dumps(protocol.error("expected hello first")))
            return

        session = ClientSession(ws, name=str(msg.get("client", "client")))
        log.info("client connected: %s %s", session.name, peer)
        await ws.send(json.dumps(self.app.make_welcome()))
        for entry in self.app.log_ring:
            await ws.send(json.dumps(entry))

        self.app.hub.sessions.add(session)
        sender = asyncio.create_task(session.sender())
        try:
            async for raw in ws:
                if isinstance(raw, (bytes, bytearray)):
                    continue                           # no binary uplink in v1
                msg, err = protocol.parse_client(raw)
                if err:
                    session.send_json(protocol.error(err))
                    continue
                self._dispatch(session, msg)
        except ConnectionClosed:
            pass
        finally:
            self.app.hub.sessions.discard(session)
            sender.cancel()
            self.app.on_client_gone(session)
            log.info("client gone: %s %s", session.name, peer)

    def _dispatch(self, session, msg):
        t = msg["type"]
        if t == "ping":
            session.send_json(protocol.pong(msg.get("t", 0.0), time.monotonic()))
        elif t == "teleop":
            self.app.on_teleop(msg.get("vx", 0.0), msg.get("wz", 0.0))
        elif t == "estop":
            self.app.on_estop(bool(msg.get("engage", True)))
        elif t == "set_mode":
            self.app.on_set_mode(str(msg.get("mode", "")), session)
        elif t == "video":
            self.app.on_video(session, msg)
        # a repeated "hello" is harmless — ignore
