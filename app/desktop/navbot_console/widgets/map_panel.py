"""MapPanel — SLAM occupancy-grid map view. Owns a MAP toggle button and a
map display as two independently placeable widgets (`.toggle` / `.view`)
rather than laying them out itself: main_window keeps the toggle in the
right-hand instrument column and swaps the (much larger) view into the main
content area on activation, so driving controls stay reachable while the
map fills the screen. The display decodes the agent-rendered PNG (robot
marker already baked in) and scales it to fit WITHOUT cropping — unlike the
camera views, a map must never lose its edges to a crop."""

import base64

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter
from PySide6.QtWidgets import QPushButton, QSizePolicy, QWidget

_STYLE_ON = "background:#00c853;color:black;font-weight:bold;"
_STYLE_OFF = ""


class _MapView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._image = None
        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_image(self, png_bytes):
        img = QImage.fromData(png_bytes, "PNG")
        if img.isNull():
            return
        self._image = img
        self.update()

    def clear(self):
        self._image = None
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(255, 255, 255))
        rect = self.rect()
        if self._image:
            scaled = self._image.scaled(rect.size(), Qt.KeepAspectRatio,
                                        Qt.SmoothTransformation)
            x = (rect.width() - scaled.width()) // 2
            y = (rect.height() - scaled.height()) // 2
            p.drawImage(x, y, scaled)
        else:
            p.setPen(QColor(120, 120, 120))
            p.drawText(rect, Qt.AlignCenter, "map\n(no map)")


class MapPanel(QObject):
    mapToggled = Signal(bool)
    saveRequested = Signal()

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
        self.view.set_image(png_bytes)

    def set_connected(self, ok):
        self.toggle.setEnabled(ok)
        self.save_btn.setEnabled(ok)
        if not ok:
            self.toggle.setChecked(False)
            self.toggle.setStyleSheet(_STYLE_OFF)
            self.view.clear()
