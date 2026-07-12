"""MainView — the primary content pane. Normally shows the full camera
grid; when the operator activates the SLAM map, the map fills this pane
instead and the camera grid becomes a small floating preview pinned to the
top-right corner, so the map gets the space it needs to be useful while
the driving controls (a separate column outside this widget) stay
reachable and the operator can still see the cameras while mapping."""

from PySide6.QtCore import QSize
from PySide6.QtWidgets import QVBoxLayout, QWidget

_PIP_SIZE = QSize(280, 220)
_PIP_MARGIN = 10


class MainView(QWidget):
    def __init__(self, video_panel, map_view, parent=None):
        super().__init__(parent)
        self.video_panel = video_panel
        self.map_view = map_view
        self._map_mode = False

        video_panel.setParent(self)
        map_view.setParent(self)
        map_view.hide()

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.addWidget(video_panel)

    def set_map_mode(self, on: bool):
        if on == self._map_mode:
            return
        self._map_mode = on
        if on:
            self._layout.removeWidget(self.video_panel)
            self._layout.addWidget(self.map_view)
            self.map_view.show()
            self.video_panel.raise_()          # float on top of the map
            self._position_pip()
        else:
            self._layout.removeWidget(self.map_view)
            self.map_view.hide()
            self._layout.addWidget(self.video_panel)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        if self._map_mode:
            self._position_pip()

    def _position_pip(self):
        w = min(_PIP_SIZE.width(), max(0, self.width() - 2 * _PIP_MARGIN))
        h = min(_PIP_SIZE.height(), max(0, self.height() - 2 * _PIP_MARGIN))
        if w <= 0 or h <= 0:
            return
        self.video_panel.setGeometry(
            self.width() - w - _PIP_MARGIN, _PIP_MARGIN, w, h)
