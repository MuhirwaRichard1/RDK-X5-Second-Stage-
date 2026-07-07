"""Video widgets: a VideoWidget renders JPEG frames letterboxed with an
optional sector HUD (the /obstacles fan) painted over the front view;
VideoPanel arranges left | front | right as equal 640x480 tiles in one row
(sides toggleable)."""

import math
import time

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QHBoxLayout, QLabel,
                               QSizePolicy, QVBoxLayout, QWidget)

_SECTOR_COLORS = {0: QColor(140, 140, 140, 110),   # UNKNOWN
                  1: QColor(0, 200, 80, 90),       # FREE
                  2: QColor(230, 40, 40, 130)}     # BLOCKED

_KIND_PIDNET, _KIND_DEPTH = 0, 1
_DETECTION_COLOR = QColor(255, 210, 0)


def _depth_color(v):
    """v: 0..255, brighter/closer -> warmer. Simple 2-stop lerp: far=blue,
    near=orange (no need for a full colormap at this grid resolution)."""
    t = max(0.0, min(1.0, v / 255.0))
    return QColor(int(30 + t * 225), int(40 + t * 120), int(120 - t * 100), 130)


class VideoWidget(QWidget):
    def __init__(self, title, hud=False, parent=None):
        super().__init__(parent)
        self._title = title
        self._hud = hud
        self._image = None
        self._sectors = None
        self._grid_overlay = None        # last GridOverlay msg (pidnet or depth)
        self._detections = []            # last Detections msg's "boxes" list
        self._frames = 0
        self._fps = 0.0
        self._fps_t0 = time.monotonic()
<<<<<<< HEAD
        self.setMinimumSize(160, 120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
=======
        self.setMinimumSize(320, 240)

    def sizeHint(self):
        return QSize(640, 480)
>>>>>>> ee971068262831a1b6350943a0f6ff02a875da3c

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
        self._grid_overlay = None
        self._detections = []
        self.update()

    def set_grid_overlay(self, msg):
        self._grid_overlay = msg
        self.update()

    def clear_grid_overlay_if_kind(self, kind):
        """Only clears if the currently-shown grid is that kind — front can
        carry pidnet and depth grids in the same slot since they're never
        both fresh at once from one toggle's perspective; this avoids a
        depthanything disable wiping a still-active pidnet overlay or
        vice versa."""
        if self._grid_overlay is not None and self._grid_overlay.get("kind") == kind:
            self._grid_overlay = None
            self.update()

    def set_detections(self, msg):
        self._detections = msg.get("boxes") or []
        self.update()

    def clear_detections(self):
        self._detections = []
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
        if self._grid_overlay:
            self._draw_grid_overlay(p)
        if self._detections:
            self._draw_detections(p)
        p.setPen(QColor(255, 255, 255))
        p.drawText(8, 18, f"{self._title}  {self._fps:.0f} fps")

    def _draw_grid_overlay(self, p):
        g = self._grid_overlay
        rows, cols, cells = g.get("rows", 0), g.get("cols", 0), g.get("cells") or []
        if rows <= 0 or cols <= 0 or len(cells) != rows * cols:
            return
        depth = g.get("kind") == _KIND_DEPTH
        cw, ch = self.width() / cols, self.height() / rows
        p.setPen(Qt.NoPen)
        for i, v in enumerate(cells):
            r, c = divmod(i, cols)
            p.setBrush(_depth_color(v) if depth else _SECTOR_COLORS.get(v, _SECTOR_COLORS[0]))
            p.drawRect(int(c * cw), int(r * ch), int(cw) + 1, int(ch) + 1)

    def _draw_detections(self, p):
        w, h = self.width(), self.height()
        box_pen = QPen(_DETECTION_COLOR, 2)
        p.setBrush(Qt.NoBrush)
        for b in self._detections:
            x1, y1 = b.get("x1", 0.0) * w, b.get("y1", 0.0) * h
            x2, y2 = b.get("x2", 0.0) * w, b.get("y2", 0.0) * h
            p.setPen(box_pen)
            p.drawRect(int(x1), int(y1), int(x2 - x1), int(y2 - y1))
            p.setPen(_DETECTION_COLOR)
            label = f'{b.get("class_name", "?")} {b.get("score", 0.0):.2f}'
            p.drawText(int(x1) + 2, max(12, int(y1) - 4), label)

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
    """Front + left + right views, equal size, side by side in one row.
    All three scale together to fill whatever space is available."""
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

<<<<<<< HEAD
        self._grid = QGridLayout()
        self._grid.addWidget(self.front, 0, 0)
        self._grid.addWidget(self.left, 0, 1)
        self._grid.addWidget(self.right, 0, 2)
=======
        # one row, cockpit order: left | front | right, equal 640x480 tiles
        grid = QGridLayout()
        grid.addWidget(self.left, 0, 0)
        grid.addWidget(self.front, 0, 1)
        grid.addWidget(self.right, 0, 2)
        for col in range(3):
            grid.setColumnStretch(col, 1)
>>>>>>> ee971068262831a1b6350943a0f6ff02a875da3c

        lay = QVBoxLayout(self)
        lay.addLayout(toggles)
        lay.addLayout(self._grid, 1)   # grid fills whatever space is left
        self._widgets = {0: self.front, 1: self.left, 2: self.right}
        self._by_name = {"front": self.front, "left": self.left, "right": self.right}
        self._sync_visibility()

    def _on_toggle(self, cam, on):
        self._sync_visibility()
        self.cameraToggled.emit(cam, on)

    def _sync_visibility(self):
        left_on = self._boxes["left"].isChecked()
        right_on = self._boxes["right"].isChecked()
        self.left.setVisible(left_on)
        self.right.setVisible(right_on)
        # hidden columns get no stretch, so the visible ones share the space
        self._grid.setColumnStretch(0, 1)
        self._grid.setColumnStretch(1, 1 if left_on else 0)
        self._grid.setColumnStretch(2, 1 if right_on else 0)

    def enabled_cams(self):
        return [c for c, b in self._boxes.items() if b.isChecked()]

    def on_frame(self, cam_id, jpeg, _seq):
        w = self._widgets.get(cam_id)
        if w:
            w.set_frame(jpeg)

    def on_sectors(self, msg):
        self.front.set_sectors(msg)

    def on_grid_overlay(self, msg):
        w = self._by_name.get(msg.get("camera"))
        if w:
            w.set_grid_overlay(msg)

    def on_detections(self, msg):
        w = self._by_name.get(msg.get("camera"))
        if w:
            w.set_detections(msg)

    def clear_model_overlay(self, model):
        """Called the moment the operator disables a toggle, so a stale
        overlay doesn't linger on screen waiting for the robot to stop
        publishing."""
        if model == "yolo11":
            for w in self._widgets.values():
                w.clear_detections()
        elif model == "pidnet":
            for w in self._widgets.values():
                w.clear_grid_overlay_if_kind(_KIND_PIDNET)
        elif model == "depthanything":
            self.front.clear_grid_overlay_if_kind(_KIND_DEPTH)

    def clear_all(self):
        for w in self._widgets.values():
            w.clear()
