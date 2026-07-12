"""MapPanel — SLAM occupancy-grid map view. Toggle button opts the session
into the agent's (bandwidth-costly, so off by default) MAP stream; the
display decodes the agent-rendered PNG (robot marker already baked in) and
scales it to fit WITHOUT cropping — unlike the camera views, a map must
never lose its edges to a crop."""

import base64

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter
from PySide6.QtWidgets import QPushButton, QSizePolicy, QVBoxLayout, QWidget

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


class MapPanel(QWidget):
    mapToggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._toggle = QPushButton("MAP")
        self._toggle.setCheckable(True)
        self._toggle.setMinimumHeight(32)
        self._toggle.clicked.connect(self._on_click)
        self._view = _MapView()

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._toggle)
        lay.addWidget(self._view, 1)
        self.set_connected(False)

    def _on_click(self):
        enabled = self._toggle.isChecked()
        self._toggle.setStyleSheet(_STYLE_ON if enabled else _STYLE_OFF)
        if not enabled:
            self._view.clear()
        self.mapToggled.emit(enabled)

    def on_map(self, msg):
        try:
            png_bytes = base64.b64decode(msg.get("png_b64", ""))
        except (ValueError, TypeError):
            return
        self._view.set_image(png_bytes)

    def set_connected(self, ok):
        self._toggle.setEnabled(ok)
        if not ok:
            self._toggle.setChecked(False)
            self._toggle.setStyleSheet(_STYLE_OFF)
            self._view.clear()
