"""WebSocket server: one connection per operator client.

Backpressure design: each session has ONE bounded outbox drained by ONE
sender task. Text messages drop-oldest when the outbox is full. Video
frames never queue — they land in a per-session latest-wins slot per
camera, with a single KICK sentinel in the outbox; a slow client simply
has its slots overwritten and memory stays bounded at one frame/camera.

UDP fast path: each session gets a random 8-byte token in its welcome; any
datagram carrying that token binds the sender's address to the session
(roaming-safe). While datagrams keep arriving, teleop rides UDP uplink and
telemetry/sectors/video ride UDP downlink — no TCP retransmit stalls, no
kernel send-buffer queueing. Silence for UDP_ALIVE_S falls back to WS."""

import asyncio
import json
import logging
import secrets
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from . import config, protocol

log = logging.getLogger("navbot.server")

_VIDEO_KICK = object()


class ClientSession:
    def __init__(self, ws, name="client"):
        self.ws = ws
        self.name = name
        self.outbox = asyncio.Queue(maxsize=256)
        self.video_slots = {}          # cam_id -> packed frame (latest wins)
        self.video_cams = set()        # camera names this client wants
        self.wants_map = False         # opted into the SLAM map view
        self._kick_pending = False
        self.token = secrets.token_bytes(8)
        self.udp_addr = None           # last address a valid datagram came from
        self.last_udp_rx = 0.0
        self._teleop_seq = 0
        self._teleop_t = 0.0
        self._estop_seq = None

    def udp_alive(self, now):
        return (self.udp_addr is not None
                and now - self.last_udp_rx < config.UDP_ALIVE_S)

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
        self.by_token = {}             # token bytes -> session
        self.udp = None                # UdpServer, set by main() when bound

    def register(self, session):
        self.sessions.add(session)
        self.by_token[session.token] = session

    def unregister(self, session):
        self.sessions.discard(session)
        self.by_token.pop(session.token, None)

    def broadcast(self, msg):
        """Reliable path (WS): state, logs, errors."""
        if self.sessions:
            text = json.dumps(msg)
            for s in list(self.sessions):
                s.send_text(text)

    def broadcast_fast(self, msg):
        """Latest-wins path: telemetry/sectors — UDP when bound, else WS."""
        text = blob = None
        now = time.monotonic()
        for s in list(self.sessions):
            if self.udp and s.udp_alive(now):
                if blob is None:
                    blob = protocol.pack_udp_json(msg)
                self.udp.sendto(blob, s.udp_addr)
            else:
                if text is None:
                    text = json.dumps(msg)
                s.send_text(text)

    def broadcast_video(self, cam_name, cam_id, seq, mono_ms, jpeg):
        packed = frags = None
        now = time.monotonic()
        for s in list(self.sessions):
            if cam_name not in s.video_cams:
                continue
            if self.udp and s.udp_alive(now):
                if frags is None:
                    frags = protocol.fragment_video(cam_id, seq, mono_ms, jpeg)
                for f in frags:
                    self.udp.sendto(f, s.udp_addr)
            else:
                if packed is None:
                    packed = protocol.pack_video(cam_id, seq, mono_ms, jpeg)
                s.send_video(cam_id, packed)

    def wants_video(self, cam_name):
        return any(cam_name in s.video_cams for s in self.sessions)

    def wants_map(self):
        return any(s.wants_map for s in self.sessions)

    def send_map(self, msg):
        """Selective send (unlike broadcast()) — the map can be a few tens
        of KB and most sessions won't have opted in."""
        if not self.sessions:
            return
        text = json.dumps(msg)
        for s in list(self.sessions):
            if s.wants_map:
                s.send_text(text)


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
        await ws.send(json.dumps(self.app.make_welcome(session)))
        for entry in self.app.log_ring:
            await ws.send(json.dumps(entry))

        self.app.hub.register(session)
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
            self.app.hub.unregister(session)
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
            self.app.on_set_mode(str(msg.get("mode", "")), session,
                                 msg.get("map"))
        elif t == "video":
            self.app.on_video(session, msg)
        elif t == "set_model":
            self.app.on_set_model(str(msg.get("model", "")),
                                  bool(msg.get("enable", True)), session)
        elif t == "set_map":
            self.app.on_set_map(session, bool(msg.get("enable", True)))
        elif t == "save_map":
            self.app.on_save_map(session, msg.get("name"))
        elif t == "set_goal":
            self.app.on_set_goal(session, msg.get("x"), msg.get("y"))
        # a repeated "hello" is harmless — ignore


class UdpServer(asyncio.DatagramProtocol):
    """UDP fast path. Every valid datagram (re)binds the session's address,
    so the client may roam APs mid-drive. Unknown tokens are dropped
    silently — this is session binding, not authentication (same trust
    model as the WS: anyone on the LAN)."""

    def __init__(self, app):
        self.app = app
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def sendto(self, data, addr):
        if self.transport:
            self.transport.sendto(data, addr)

    def datagram_received(self, data, addr):
        if len(data) < 9:
            return
        session = self.app.hub.by_token.get(data[1:9])
        if session is None:
            return
        now = time.monotonic()
        session.udp_addr = addr
        session.last_udp_rx = now
        magic = data[0]
        if magic == protocol.UDP_PING and len(data) == protocol.UDP_PING_S.size:
            _, _, t = protocol.UDP_PING_S.unpack(data)
            self.sendto(protocol.UDP_PONG_S.pack(protocol.UDP_PONG, t, now), addr)
        elif magic == protocol.UDP_TELEOP and len(data) == protocol.UDP_TELEOP_S.size:
            _, _, seq, vx, wz = protocol.UDP_TELEOP_S.unpack(data)
            # UDP reorders: only ever act on the newest command. A >1 s gap
            # resets the window (client restarted its counter).
            if seq > session._teleop_seq or now - session._teleop_t > 1.0:
                session._teleop_seq = seq
                session._teleop_t = now
                self.app.on_teleop(vx, wz)
        elif magic == protocol.UDP_ESTOP and len(data) == protocol.UDP_ESTOP_S.size:
            _, _, seq, engage = protocol.UDP_ESTOP_S.unpack(data)
            if seq != session._estop_seq:      # client bursts x3 per press
                session._estop_seq = seq
                self.app.on_estop(bool(engage))
