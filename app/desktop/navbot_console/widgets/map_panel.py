"""MapPanel — SLAM occupancy-grid map view. Owns a MAP toggle, a SAVE MAP
button, and a map display as independently placeable widgets (`.toggle` /
`.save_btn` / `.view`) rather than laying them out itself: main_window keeps
the buttons in the right-hand instrument column and swaps the (much larger)
view into the main content area on activation, so driving controls stay
reachable while the map fills the screen. The display decodes the agent-
rendered PNG (robot marker already baked in) and scales it to fit WITHOUT
cropping — a map must never lose its edges to a crop.

Clicking the map (in NAVIGATE mode) picks a goal: the click pixel is turned
into a map-frame (x, y) using the resolution/origin the agent sends with each
map, and emitted as goalPicked. The chosen goal is drawn as a green cross that
tracks the map as it pans/grows."""

import base64

from PySide6.QtCore import QObject, QRect, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import QPushButton, QSizePolicy, QWidget

_STYLE_ON = "background:#00c853;color:black;font-weight:bold;"
_STYLE_OFF = ""
_GOAL_COLOR = QColor(0, 200, 0)

# map->base_link fix older than this (ms) = SLAM lost track (agent POSE_STALE_S).
_POSE_STALE_MS = 1500
_ODOM_LABELS = {"icp": "icp", "dr": "dr", "fused": "fused"}


class _MapView(QWidget):
    clicked = Signal(float, float)          # image-pixel (col, row)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._image = None
        self._draw = None                   # (x_off, y_off, sw, sh, img_w, img_h)
        self._goal_px = None                # (col, row) image-pixel goal marker
        self._source = None                 # live SLAM odom backbone, or None
        self._pose_age_ms = None            # age of last fix; None = never localized
        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_image(self, png_bytes):
        img = QImage.fromData(png_bytes, "PNG")
        if img.isNull():
            return
        self._image = img
        self.update()

    def set_goal_px(self, colrow):
        self._goal_px = colrow
        self.update()

    def set_loc(self, source, pose_age_ms):
        self._source, self._pose_age_ms = source, pose_age_ms
        if self._image:
            self.update()

    def clear(self):
        self._image = None
        self._draw = None
        self._goal_px = None
        self.update()

    def _loc_badge(self):
        """(text, color) when localization needs flagging, else None."""
        if not self._source:
            return None                      # no SLAM odom running — no map anyway
        name = _ODOM_LABELS.get(self._source, self._source)
        if self._pose_age_ms is None:
            return f"{name} · NO FIX", QColor(255, 179, 0)
        if self._pose_age_ms > _POSE_STALE_MS:
            return f"{name} · POSE LOST", QColor(229, 57, 53)
        return None                          # localized — marker + health row say it

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(255, 255, 255))
        rect = self.rect()
        if not self._image:
            p.setPen(QColor(110, 110, 110))
            f = QFont()
            f.setPointSize(13)
            f.setBold(True)
            p.setFont(f)
            p.drawText(rect, Qt.AlignCenter,
                       "No map yet\n\nStart MAPPING (or NAVIGATE) mode on the\n"
                       "robot — the map appears here as it builds.")
            self._draw = None
            return
        scaled = self._image.scaled(rect.size(), Qt.KeepAspectRatio,
                                    Qt.SmoothTransformation)
        x = (rect.width() - scaled.width()) // 2
        y = (rect.height() - scaled.height()) // 2
        p.drawImage(x, y, scaled)
        self._draw = (x, y, scaled.width(), scaled.height(),
                      self._image.width(), self._image.height())
        if self._goal_px is not None:
            gx, gy = self._to_widget(*self._goal_px)
            p.setPen(QPen(_GOAL_COLOR, 2))
            p.drawLine(gx - 6, gy, gx + 6, gy)
            p.drawLine(gx, gy - 6, gx, gy + 6)
        badge = self._loc_badge()
        if badge:
            self._paint_badge(p, rect, *badge)

    def _paint_badge(self, p, rect, text, color):
        f = QFont()
        f.setBold(True)
        f.setPointSize(11)
        p.setFont(f)
        fm = p.fontMetrics()
        chip = QRect(rect.left() + 8, rect.top() + 8,
                     fm.horizontalAdvance(text) + 16, fm.height() + 8)
        p.fillRect(chip, color)
        p.setPen(QColor(255, 255, 255))
        p.drawText(chip, Qt.AlignCenter, text)

    def _to_widget(self, col, row):
        x_off, y_off, sw, sh, iw, ih = self._draw
        return int(x_off + col * sw / iw), int(y_off + row * sh / ih)

    def mousePressEvent(self, ev):
        if self._draw is None:
            return
        x_off, y_off, sw, sh, iw, ih = self._draw
        pos = ev.position()
        mx, my = pos.x(), pos.y()
        if not (x_off <= mx < x_off + sw and y_off <= my < y_off + sh):
            return
        col = (mx - x_off) * iw / sw
        row = (my - y_off) * ih / sh
        self.clicked.emit(col, row)


class MapPanel(QObject):
    mapToggled = Signal(bool)
    saveRequested = Signal()
    goalPicked = Signal(float, float)       # map-frame (x, y)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.toggle = QPushButton("MAP")
        self.toggle.setCheckable(True)
        self.toggle.setMinimumHeight(32)
        self.toggle.clicked.connect(self._on_click)
        # Save the live SLAM map (needs a mapping mode running on the robot).
        self.save_btn = QPushButton("SAVE MAP")
        self.save_btn.setMinimumHeight(28)
        self.save_btn.clicked.connect(lambda: self.saveRequested.emit())
        self.view = _MapView()
        self.view.clicked.connect(self._on_view_click)
        self._meta = None                   # (resolution, origin_x, origin_y)
        self._goal_world = None             # (x, y) chosen goal, map frame
        self.set_connected(False)

    def _on_click(self):
        enabled = self.toggle.isChecked()
        self.toggle.setStyleSheet(_STYLE_ON if enabled else _STYLE_OFF)
        if not enabled:
            self.view.clear()
        self.mapToggled.emit(enabled)

    def on_map(self, msg):
        try:
            png_bytes = base64.b64decode(msg.get("png_b64", ""))
        except (ValueError, TypeError):
            return
        res = msg.get("resolution", 0.0)
        if res:                             # older agent may omit metadata
            self._meta = (res, msg.get("origin_x", 0.0), msg.get("origin_y", 0.0))
        self.view.set_image(png_bytes)
        self._refresh_goal_marker()

    def on_telemetry(self, t):
        """Drive the map's lost-track badge from the 2 Hz telemetry (always
        fresh) rather than the map frame, which only ships when the grid
        changes — a frozen map must still flag POSE LOST promptly."""
        odom = t.get("odom") or {}
        self.view.set_loc(odom.get("source"), odom.get("pose_age_ms"))

    def _on_view_click(self, col, row):
        """Image-pixel click -> map-frame (x, y). The PNG is flipped so
        image-up = world +y (map_pump): row 0 is the top = max y."""
        if self._meta is None or self.view._image is None:
            return
        res, ox, oy = self._meta
        ih = self.view._image.height()
        x = ox + col * res
        y = oy + (ih - 1 - row) * res
        self._goal_world = (x, y)
        self._refresh_goal_marker()
        self.goalPicked.emit(x, y)

    def _refresh_goal_marker(self):
        if self._goal_world is None or self._meta is None \
                or self.view._image is None:
            return
        res, ox, oy = self._meta
        ih = self.view._image.height()
        col = (self._goal_world[0] - ox) / res
        row = (ih - 1) - (self._goal_world[1] - oy) / res
        self.view.set_goal_px((col, row))

    def set_connected(self, ok):
        self.toggle.setEnabled(ok)
        self.save_btn.setEnabled(ok)
        if not ok:
            self.toggle.setChecked(False)
            self.toggle.setStyleSheet(_STYLE_OFF)
            self._goal_world = None
            self._meta = None
            self.view.set_loc(None, None)
            self.view.clear()
