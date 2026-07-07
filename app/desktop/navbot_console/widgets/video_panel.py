"""Video widgets: a VideoWidget renders JPEG frames letterboxed with an
optional sector HUD (the /obstacles fan) painted over the front view;
VideoPanel arranges left | front | right as equal 640x480 tiles in one row
(sides toggleable)."""

import math
import time

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QHBoxLayout, QLabel,
                               QVBoxLayout, QWidget)

_SECTOR_COLORS = {0: QColor(140, 140, 140, 110),   # UNKNOWN
                  1: QColor(0, 200, 80, 90),       # FREE
                  2: QColor(230, 40, 40, 130)}     # BLOCKED


class VideoWidget(QWidget):
    def __init__(self, title, hud=False, parent=None):
        super().__init__(parent)
        self._title = title
        self._hud = hud
        self._image = None
        self._sectors = None
        self._frames = 0
        self._fps = 0.0
        self._fps_t0 = time.monotonic()
        self.setMinimumSize(320, 240)

    def sizeHint(self):
        return QSize(640, 480)

    def set_frame(self, jpeg: bytes):
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        self._image = img
        self._frames += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 1.0:
            self._fps = self._frames / (now - self._fps_t0)
            self._frames, self._fps_t0 = 0, now
        self.update()

    def set_sectors(self, msg: dict):
        self._sectors = msg
        if self._hud:
            self.update()

    def clear(self):
        self._image = None
        self.update()

    # ---------------- painting ----------------

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(15, 15, 15))
        rect = self.rect()
        if self._image:
            scaled = self._image.scaled(rect.size(), Qt.KeepAspectRatio,
                                        Qt.SmoothTransformation)
            x = (rect.width() - scaled.width()) // 2
            y = (rect.height() - scaled.height()) // 2
            p.drawImage(x, y, scaled)
        else:
            p.setPen(QColor(120, 120, 120))
            p.drawText(rect, Qt.AlignCenter, f"{self._title}\n(no video)")
        if self._hud and self._sectors:
            self._draw_sectors(p)
        p.setPen(QColor(255, 255, 255))
        p.drawText(8, 18, f"{self._title}  {self._fps:.0f} fps")

    def _draw_sectors(self, p):
        """Fan of wedges, robot at bottom-center, bearing 0 = up (ahead).
        Robot-left (+bearing, CCW) renders to screen-left."""
        s = self._sectors
        status = s.get("status") or []
        if not status:
            return
        n = len(status)
        a0, a1 = s["angle_min"], s["angle_max"]
        width = (a1 - a0) / n
        cx = self.width() / 2
        cy = self.height() - 10
        r = int(min(self.width(), self.height()) * 0.38)
        box = (int(cx - r), int(cy - r), 2 * r, 2 * r)
        p.setPen(Qt.NoPen)
        for i, st in enumerate(status):
            # sector bearing (rad, robot frame) -> Qt pie angle (1/16 deg,
            # 0 = 3 o'clock, CCW+). Screen angle = 90 + bearing_deg because
            # +bearing (robot left) must appear left of straight-up.
            b_start = a0 + i * width
            start_qt = int((90 + math.degrees(b_start)) * 16)
            span_qt = int(math.degrees(width) * 16)
            p.setBrush(_SECTOR_COLORS.get(st, _SECTOR_COLORS[0]))
            p.drawPie(*box, start_qt, span_qt)
        p.setPen(QPen(QColor(255, 255, 255, 150), 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(int(cx - 4), int(cy - 4), 8, 8)


class VideoPanel(QWidget):
    """Front view + toggleable side views; emits camera subscribe requests."""
    cameraToggled = Signal(str, bool)        # cam name, enabled

    def __init__(self, parent=None):
        super().__init__(parent)
        self.front = VideoWidget("front", hud=True)
        self.left = VideoWidget("left")
        self.right = VideoWidget("right")

        self._boxes = {}
        toggles = QHBoxLayout()
        toggles.addWidget(QLabel("cameras:"))
        for cam in ("front", "left", "right"):
            box = QCheckBox(cam)
            box.setChecked(cam == "front")
            box.toggled.connect(lambda on, c=cam: self._on_toggle(c, on))
            toggles.addWidget(box)
            self._boxes[cam] = box
        toggles.addStretch(1)

        # one row, cockpit order: left | front | right, equal 640x480 tiles
        grid = QGridLayout()
        grid.addWidget(self.left, 0, 0)
        grid.addWidget(self.front, 0, 1)
        grid.addWidget(self.right, 0, 2)
        for col in range(3):
            grid.setColumnStretch(col, 1)

        lay = QVBoxLayout(self)
        lay.addLayout(toggles)
        lay.addLayout(grid)
        self._widgets = {0: self.front, 1: self.left, 2: self.right}
        self._sync_visibility()

    def _on_toggle(self, cam, on):
        self._sync_visibility()
        self.cameraToggled.emit(cam, on)

    def _sync_visibility(self):
        self.left.setVisible(self._boxes["left"].isChecked())
        self.right.setVisible(self._boxes["right"].isChecked())

    def enabled_cams(self):
        return [c for c, b in self._boxes.items() if b.isChecked()]

    def on_frame(self, cam_id, jpeg, _seq):
        w = self._widgets.get(cam_id)
        if w:
            w.set_frame(jpeg)

    def on_sectors(self, msg):
        self.front.set_sectors(msg)

    def clear_all(self):
        for w in self._widgets.values():
            w.clear()
