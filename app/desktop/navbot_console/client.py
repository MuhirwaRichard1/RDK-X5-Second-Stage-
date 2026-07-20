"""RobotClient — QWebSocket wrapper speaking the navbot agent protocol v1,
plus the UDP fast path (teleop up; telemetry/sectors/video down) the agent
advertises in its welcome message — same split a drone GCS uses.

Emits Qt signals for every message class; owns the 1 Hz ping (RTT), the
auto-reconnect backoff, and a staleness watchdog. All timestamps are local
monotonic — robot and PC clocks are never compared. Transport choice is
automatic: teleop rides UDP while a UDP pong/datagram arrived in the last
2.5 s, otherwise everything falls back to the WebSocket. E-stop is always
sent on BOTH channels (UDP burst x3 + WS)."""

import json
import socket
import struct
import time

from PySide6.QtCore import QObject, QTimer, QUrl, Signal
from PySide6.QtNetwork import QHostAddress, QUdpSocket
from PySide6.QtWebSockets import QWebSocket

PROTO = 1
_VIDEO_HDR = struct.Struct(">BBII")
_BACKOFF_S = (1.0, 2.0, 5.0)

# UDP fast path (see docs/operator_app.md): uplink [magic][token 8B]...
_UDP_PING_S = struct.Struct(">B8sd")
_UDP_TELEOP_S = struct.Struct(">B8sIff")
_UDP_ESTOP_S = struct.Struct(">B8sIB")
_UDP_PONG_S = struct.Struct(">Bdd")
_UDP_VIDEO_S = struct.Struct(">BBHIBB")
_UDP_PING_M, _UDP_TELEOP_M, _UDP_ESTOP_M = 0x10, 0x11, 0x12
_UDP_PONG_M, _UDP_JSON_M, _UDP_VIDEO_M = 0x20, 0x21, 0x22
_UDP_ALIVE_S = 2.5


def _newer16(a, b):
    """True if u16 sequence a is newer than b (wraparound-aware)."""
    return 0 < ((a - b) & 0xFFFF) < 0x8000


class RobotClient(QObject):
    connectedChanged = Signal(bool)          # socket state
    staleChanged = Signal(bool)              # no traffic for > 3 s
    welcomeReceived = Signal(dict)
    stateReceived = Signal(dict)
    telemetryReceived = Signal(dict)
    sectorsReceived = Signal(dict)
    gridOverlayReceived = Signal(dict)
    detectionsReceived = Signal(dict)
    mapReceived = Signal(dict)
    attitudeReceived = Signal(dict)          # 10 Hz roll/pitch/yaw/yaw_rate
    logReceived = Signal(dict)
    errorReceived = Signal(str)
    videoFrame = Signal(int, bytes, int)     # cam_id, jpeg, seq
    latencyMs = Signal(float)
    transportChanged = Signal(str)           # "UDP" | "TCP"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ws = QWebSocket()
        self._ws.connected.connect(self._on_connected)
        self._ws.disconnected.connect(self._on_disconnected)
        self._ws.textMessageReceived.connect(self._on_text)
        self._ws.binaryMessageReceived.connect(self._on_binary)

        self._udp = QUdpSocket(self)
        self._udp.readyRead.connect(self._on_udp_ready)
        self._udp_token = None               # 8B from welcome; None = WS only
        self._udp_addr = None                # QHostAddress of the agent
        self._udp_port = 0
        self._last_udp_rx = 0.0
        self._estop_seq = 0
        self._frags = {}                     # cam_id -> [seq, nfrags, {idx: chunk}]
        self._transport = "TCP"

        self._url = None
        self._want_connection = False
        self._backoff_i = 0
        self._connected = False
        self._last_rx = 0.0
        self._stale = False

        self._ping_timer = QTimer(self, interval=1000)
        self._ping_timer.timeout.connect(self._ping)
        self._watch_timer = QTimer(self, interval=1000)
        self._watch_timer.timeout.connect(self._watchdog)

    # ---------------- public API ----------------

    @property
    def connected(self):
        return self._connected

    @property
    def udp_alive(self):
        return (self._udp_token is not None
                and time.monotonic() - self._last_udp_rx < _UDP_ALIVE_S)

    def open(self, url: str):
        self._url = url
        self._want_connection = True
        self._backoff_i = 0
        self._ws.abort()
        self._ws.open(QUrl(url))

    def close(self):
        self._want_connection = False
        self._ws.close()

    def send(self, msg: dict):
        if self._connected:
            self._ws.sendTextMessage(json.dumps(msg))

    def send_teleop(self, vx, wz, seq=0):
        if self._connected and self.udp_alive:
            self._udp.writeDatagram(
                _UDP_TELEOP_S.pack(_UDP_TELEOP_M, self._udp_token,
                                   seq & 0xFFFFFFFF, vx, wz),
                self._udp_addr, self._udp_port)
        else:
            self.send({"type": "teleop", "vx": vx, "wz": wz, "seq": seq})

    def send_estop(self, engage):
        # Fast trigger over UDP (x3 against loss), authoritative copy over WS.
        if self._udp_token is not None and self._udp_addr is not None:
            self._estop_seq += 1
            pkt = _UDP_ESTOP_S.pack(_UDP_ESTOP_M, self._udp_token,
                                    self._estop_seq & 0xFFFFFFFF, int(bool(engage)))
            for _ in range(3):
                self._udp.writeDatagram(pkt, self._udp_addr, self._udp_port)
        self.send({"type": "estop", "engage": bool(engage)})

    def send_mode(self, mode):
        self.send({"type": "set_mode", "mode": mode})

    def send_model(self, model, enable):
        self.send({"type": "set_model", "model": model, "enable": bool(enable)})

    def send_map(self, enable):
        self.send({"type": "set_map", "enable": bool(enable)})

    def send_save_map(self, name=None):
        msg = {"type": "save_map"}
        if name:
            msg["name"] = name
        self.send(msg)

    def send_goal(self, x, y):
        self.send({"type": "set_goal", "x": float(x), "y": float(y)})

    def send_video(self, cam, enable, fps=None, quality=None):
        msg = {"type": "video", "cam": cam, "enable": bool(enable)}
        if fps:
            msg["fps"] = fps
        if quality:
            msg["quality"] = quality
        self.send(msg)

    # ---------------- socket events ----------------

    def _on_connected(self):
        self._connected = True
        self._backoff_i = 0
        self._last_rx = time.monotonic()
        self._ws.sendTextMessage(json.dumps(
            {"v": PROTO, "type": "hello", "client": "navbot-console/0.1"}))
        self._ping_timer.start()
        self._watch_timer.start()
        self.connectedChanged.emit(True)

    def _on_disconnected(self):
        was = self._connected
        self._connected = False
        self._udp_token = None               # token dies with the WS session
        self._frags.clear()
        self._set_transport("TCP")
        self._ping_timer.stop()
        self._watch_timer.stop()
        if was:
            self.connectedChanged.emit(False)
        if self._want_connection:
            delay = _BACKOFF_S[min(self._backoff_i, len(_BACKOFF_S) - 1)]
            self._backoff_i += 1
            QTimer.singleShot(int(delay * 1000), self._reconnect)

    def _reconnect(self):
        if self._want_connection and not self._connected:
            self._ws.abort()
            self._ws.open(QUrl(self._url))

    # ---------------- rx ----------------

    def _on_text(self, text):
        self._mark_rx()
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            return
        if isinstance(msg, dict):
            self._handle_msg(msg)

    def _handle_msg(self, msg):
        t = msg.get("type")
        if t == "welcome":
            if msg.get("proto") != PROTO:
                self.errorReceived.emit(
                    f"protocol mismatch: agent={msg.get('proto')} console={PROTO}")
                self.close()
                return
            self._setup_udp(msg.get("udp"))
            self.welcomeReceived.emit(msg)
            if "state" in msg:
                self.stateReceived.emit(msg["state"])
        elif t == "state":
            self.stateReceived.emit(msg)
        elif t == "telemetry":
            self.telemetryReceived.emit(msg)
        elif t == "sectors":
            self.sectorsReceived.emit(msg)
        elif t == "grid_overlay":
            self.gridOverlayReceived.emit(msg)
        elif t == "detections":
            self.detectionsReceived.emit(msg)
        elif t == "map":
            self.mapReceived.emit(msg)
        elif t == "att":
            self.attitudeReceived.emit(msg)
        elif t == "log":
            self.logReceived.emit(msg)
        elif t == "pong":
            if not self.udp_alive:           # UDP RTT wins while the path is up
                self._set_transport("TCP")
                self.latencyMs.emit((time.monotonic() - msg.get("t", 0.0)) * 1000)
        elif t == "error":
            self.errorReceived.emit(str(msg.get("msg")))

    def _on_binary(self, data):
        self._mark_rx()
        raw = bytes(data)
        if len(raw) <= _VIDEO_HDR.size:
            return
        magic, cam_id, seq, _mono = _VIDEO_HDR.unpack(raw[:_VIDEO_HDR.size])
        if magic == 0x01:
            self.videoFrame.emit(cam_id, raw[_VIDEO_HDR.size:], seq)

    # ---------------- UDP fast path ----------------

    def _setup_udp(self, info):
        self._udp_token = None
        self._last_udp_rx = 0.0
        if not isinstance(info, dict):
            return                           # older agent: WS only
        try:
            token = bytes.fromhex(str(info.get("token", "")))
            port = int(info.get("port", 0))
        except (TypeError, ValueError):
            return
        if len(token) != 8 or not 0 < port < 65536:
            return
        host = QUrl(self._url).host()
        addr = QHostAddress(host)
        if addr.isNull():                    # hostname, not a literal IP
            try:
                addr = QHostAddress(socket.getaddrinfo(host, None)[0][4][0])
            except (socket.gaierror, IndexError, OSError):
                return
        self._udp_token, self._udp_addr, self._udp_port = token, addr, port
        self._udp_ping()                     # bind the agent side right away

    def _udp_ping(self):
        self._udp.writeDatagram(
            _UDP_PING_S.pack(_UDP_PING_M, self._udp_token, time.monotonic()),
            self._udp_addr, self._udp_port)

    def _on_udp_ready(self):
        while self._udp.hasPendingDatagrams():
            raw = bytes(self._udp.receiveDatagram().data())
            if not raw:
                continue
            magic = raw[0]
            if magic == _UDP_PONG_M and len(raw) == _UDP_PONG_S.size:
                self._mark_udp_rx()
                _, t, _agent_t = _UDP_PONG_S.unpack(raw)
                self._set_transport("UDP")
                self.latencyMs.emit((time.monotonic() - t) * 1000)
            elif magic == _UDP_JSON_M:
                self._mark_udp_rx()
                try:
                    msg = json.loads(raw[1:].decode())
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if isinstance(msg, dict):
                    self._handle_msg(msg)
            elif magic == _UDP_VIDEO_M and len(raw) > _UDP_VIDEO_S.size:
                self._mark_udp_rx()
                _, cam_id, seq, _mono, idx, nfrags = \
                    _UDP_VIDEO_S.unpack(raw[:_UDP_VIDEO_S.size])
                self._on_video_frag(cam_id, seq, nfrags, idx,
                                    raw[_UDP_VIDEO_S.size:])

    def _on_video_frag(self, cam_id, seq, nfrags, idx, chunk):
        """Reassemble; a newer frame for the same camera discards a partial
        one (lost fragment = one dropped frame, never a stall)."""
        st = self._frags.get(cam_id)
        if st is None or _newer16(seq, st[0]):
            st = self._frags[cam_id] = [seq, nfrags, {}]
        elif seq != st[0] or nfrags != st[1]:
            return                           # stale or inconsistent fragment
        if idx >= nfrags:
            return
        st[2][idx] = chunk
        if len(st[2]) == nfrags:
            del self._frags[cam_id]
            self.videoFrame.emit(cam_id, b"".join(st[2][i] for i in range(nfrags)), seq)

    def _mark_udp_rx(self):
        self._last_udp_rx = time.monotonic()
        self._mark_rx()

    def _set_transport(self, name):
        if name != self._transport:
            self._transport = name
            self.transportChanged.emit(name)

    # ---------------- ping / staleness ----------------

    def _ping(self):
        self.send({"type": "ping", "t": time.monotonic()})
        if self._udp_token is not None:
            self._udp_ping()

    def _mark_rx(self):
        self._last_rx = time.monotonic()
        if self._stale:
            self._stale = False
            self.staleChanged.emit(False)

    def _watchdog(self):
        stale = time.monotonic() - self._last_rx > 3.0
        if stale != self._stale:
            self._stale = stale
            self.staleChanged.emit(stale)
