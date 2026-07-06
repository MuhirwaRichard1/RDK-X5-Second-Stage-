"""RobotClient — QWebSocket wrapper speaking the navbot agent protocol v1.

Emits Qt signals for every message class; owns the 1 Hz ping (RTT), the
auto-reconnect backoff, and a staleness watchdog. All timestamps are local
monotonic — robot and PC clocks are never compared."""

import json
import struct
import time

from PySide6.QtCore import QObject, QTimer, QUrl, Signal
from PySide6.QtWebSockets import QWebSocket

PROTO = 1
_VIDEO_HDR = struct.Struct(">BBII")
_BACKOFF_S = (1.0, 2.0, 5.0)


class RobotClient(QObject):
    connectedChanged = Signal(bool)          # socket state
    staleChanged = Signal(bool)              # no traffic for > 3 s
    welcomeReceived = Signal(dict)
    stateReceived = Signal(dict)
    telemetryReceived = Signal(dict)
    sectorsReceived = Signal(dict)
    logReceived = Signal(dict)
    errorReceived = Signal(str)
    videoFrame = Signal(int, bytes, int)     # cam_id, jpeg, seq
    latencyMs = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ws = QWebSocket()
        self._ws.connected.connect(self._on_connected)
        self._ws.disconnected.connect(self._on_disconnected)
        self._ws.textMessageReceived.connect(self._on_text)
        self._ws.binaryMessageReceived.connect(self._on_binary)

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
        self.send({"type": "teleop", "vx": vx, "wz": wz, "seq": seq})

    def send_estop(self, engage):
        self.send({"type": "estop", "engage": bool(engage)})

    def send_mode(self, mode):
        self.send({"type": "set_mode", "mode": mode})

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
        t = msg.get("type")
        if t == "welcome":
            if msg.get("proto") != PROTO:
                self.errorReceived.emit(
                    f"protocol mismatch: agent={msg.get('proto')} console={PROTO}")
                self.close()
                return
            self.welcomeReceived.emit(msg)
            if "state" in msg:
                self.stateReceived.emit(msg["state"])
        elif t == "state":
            self.stateReceived.emit(msg)
        elif t == "telemetry":
            self.telemetryReceived.emit(msg)
        elif t == "sectors":
            self.sectorsReceived.emit(msg)
        elif t == "log":
            self.logReceived.emit(msg)
        elif t == "pong":
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

    # ---------------- ping / staleness ----------------

    def _ping(self):
        self.send({"type": "ping", "t": time.monotonic()})

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
