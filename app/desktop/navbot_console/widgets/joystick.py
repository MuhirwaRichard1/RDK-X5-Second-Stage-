"""JoystickWidget — circular drag pad. Emits moved(x, y), x right+, y up+,
both in -1..1; springs back to center (0,0) on release."""

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget


class JoystickWidget(QWidget):
    moved = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self._knob = QPointF(0, 0)          # normalized
        self._dragging = False

    # ---------------- geometry ----------------

    def _radius(self):
        return min(self.width(), self.height()) / 2 - 10

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _to_norm(self, pos):
        r = self._radius()
        d = QPointF(pos) - self._center()
        x, y = d.x() / r, d.y() / r
        mag = (x * x + y * y) ** 0.5
        if mag > 1.0:
            x, y = x / mag, y / mag
        return x, -y                         # screen y down -> up positive

    # ---------------- mouse ----------------

    def mousePressEvent(self, ev):
        self._dragging = True
        self._update_knob(ev.position())

    def mouseMoveEvent(self, ev):
        if self._dragging:
            self._update_knob(ev.position())

    def mouseReleaseEvent(self, _ev):
        self._dragging = False
        self._knob = QPointF(0, 0)
        self.moved.emit(0.0, 0.0)
        self.update()

    def _update_knob(self, pos):
        x, y = self._to_norm(pos)
        self._knob = QPointF(x, y)
        self.moved.emit(x, y)
        self.update()

    # ---------------- paint ----------------

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c, r = self._center(), self._radius()
        p.setPen(QPen(QColor(120, 120, 120), 2))
        p.setBrush(QColor(40, 40, 40))
        p.drawEllipse(c, r, r)
        p.setPen(QPen(QColor(80, 80, 80), 1, Qt.DashLine))
        p.drawLine(c.x() - r, c.y(), c.x() + r, c.y())
        p.drawLine(c.x(), c.y() - r, c.x(), c.y() + r)
        knob = QPointF(c.x() + self._knob.x() * r, c.y() - self._knob.y() * r)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 170, 255) if self._dragging else QColor(130, 130, 130))
        p.drawEllipse(knob, r * 0.22, r * 0.22)
