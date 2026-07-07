"""AttitudePanel — GCS-style instruments fed by the agent's 10 Hz `att`
stream (MPU6050 complementary filter on the robot).

ArtificialHorizon: roll/pitch (aviation convention: bank right +, nose up +).
HeadingDial: gyro-integrated heading — RELATIVE to power-on and drifts
slowly (no magnetometer on the MPU6050), still ideal for judging turns.
Both grey out with a NO IMU overlay when the stream goes stale."""

import time

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

SKY = QColor("#2f6fae")
GROUND = QColor("#7a5230")
BEZEL = QColor("#101820")
FACE = QColor("#182430")
LINE = QColor("#e8f1f8")
ACCENT = QColor("#3ec7ee")
WING = QColor("#f4c542")
STALE = QColor(16, 24, 32, 200)

_PX_PER_DEG = 1.8            # pitch scale, design units (200 x 200 canvas)


class _Instrument(QWidget):
    """Square QPainter canvas in 200x200 design units, with stale overlay."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stale = True
        self.setMinimumSize(120, 120)

    def sizeHint(self):
        return self.minimumSize()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        side = min(self.width(), self.height())
        p.translate(self.width() / 2, self.height() / 2)
        p.scale(side / 200.0, side / 200.0)
        self.draw(p)
        if self.stale:
            p.setBrush(STALE)
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(0, 0), 97, 97)
            p.setPen(QPen(QColor("#8aa0b4")))
            p.setFont(QFont("", 16, QFont.Bold))
            p.drawText(QRectF(-90, -20, 180, 40), Qt.AlignCenter, "NO IMU")
        p.end()

    def draw(self, p):                       # overridden
        raise NotImplementedError


class ArtificialHorizon(_Instrument):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0

    def set_attitude(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def draw(self, p):
        clip = QPainterPath()
        clip.addEllipse(QPointF(0, 0), 95, 95)
        p.save()
        p.setClipPath(clip)
        p.rotate(-self.roll)
        off = self.pitch * _PX_PER_DEG      # nose up -> horizon slides down
        p.fillRect(QRectF(-300, -300, 600, 300 + off), SKY)
        p.fillRect(QRectF(-300, off, 600, 300), GROUND)
        p.setPen(QPen(LINE, 2.5))
        p.drawLine(QPointF(-300, off), QPointF(300, off))

        p.setFont(QFont("", 7))
        for d in (-30, -20, -10, 10, 20, 30):
            y = off + d * _PX_PER_DEG       # +d ladder line sits above horizon
            w = 18 + abs(d)
            p.setPen(QPen(LINE, 1.2))
            p.drawLine(QPointF(-w, y), QPointF(w, y))
            p.drawText(QRectF(w + 2, y - 7, 24, 14),
                       Qt.AlignLeft | Qt.AlignVCenter, f"{abs(d)}")
        p.restore()

        # roll scale (fixed) + pointer at current bank
        p.setPen(QPen(LINE, 2))
        for a in (-45, -30, -20, -10, 0, 10, 20, 30, 45):
            p.save()
            p.rotate(a)
            p.drawLine(QPointF(0, -95), QPointF(0, -87 if a else -83))
            p.restore()
        p.save()
        p.rotate(-self.roll)
        p.setBrush(WING)
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -84), QPointF(-6, -70),
                                 QPointF(6, -70)]))
        p.restore()

        # fixed aircraft symbol
        p.setPen(QPen(WING, 4, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(-52, 0), QPointF(-18, 0))
        p.drawLine(QPointF(18, 0), QPointF(52, 0))
        p.drawLine(QPointF(-18, 0), QPointF(-8, 8))
        p.drawLine(QPointF(18, 0), QPointF(8, 8))
        p.setBrush(WING)
        p.drawEllipse(QPointF(0, 0), 2.5, 2.5)

        p.setPen(QPen(BEZEL, 6))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0, 0), 95, 95)


class HeadingDial(_Instrument):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0
        self.yaw_rate = 0.0

    def set_heading(self, heading, yaw_rate):
        self.heading, self.yaw_rate = heading, yaw_rate
        self.update()

    def draw(self, p):
        p.setBrush(FACE)
        p.setPen(QPen(BEZEL, 6))
        p.drawEllipse(QPointF(0, 0), 95, 95)

        p.save()
        p.rotate(-self.heading)             # card rotates, lubber line fixed
        for deg in range(0, 360, 10):
            p.save()
            p.rotate(deg)
            major = deg % 30 == 0
            p.setPen(QPen(ACCENT if deg == 0 else LINE, 2 if major else 1))
            p.drawLine(QPointF(0, -90), QPointF(0, -80 if major else -85))
            if major:
                label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(
                    deg, str(deg // 10))
                p.setFont(QFont("", 9 if label in "NESW" else 7,
                                QFont.Bold if label in "NESW" else QFont.Normal))
                p.drawText(QRectF(-12, -78, 24, 14), Qt.AlignCenter, label)
            p.restore()
        p.restore()

        p.setBrush(ACCENT)                  # fixed lubber triangle
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0, -92), QPointF(-6, -104),
                                 QPointF(6, -104)]))

        p.setPen(QPen(LINE))
        p.setFont(QFont("", 17, QFont.Bold))
        p.drawText(QRectF(-50, -14, 100, 26), Qt.AlignCenter,
                   f"{self.heading:03.0f}\N{DEGREE SIGN}")
        p.setFont(QFont("", 7))
        p.setPen(QPen(QColor("#8aa0b4")))
        p.drawText(QRectF(-50, 12, 100, 12), Qt.AlignCenter,
                   f"rel \N{BULLET} {self.yaw_rate:+.0f}\N{DEGREE SIGN}/s")


class AttitudePanel(QWidget):
    """Group box holding both instruments + a numeric readout row."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.horizon = ArtificialHorizon()
        self.dial = HeadingDial()
        self._values = QLabel("—")
        self._values.setAlignment(Qt.AlignCenter)
        self._last_rx = 0.0

        box = QGroupBox("attitude (IMU)")
        row = QHBoxLayout()
        row.addWidget(self.horizon, 1)
        row.addWidget(self.dial, 1)
        v = QVBoxLayout(box)
        v.addLayout(row)
        v.addWidget(self._values)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(box)

        self._watch = QTimer(self, interval=500)
        self._watch.timeout.connect(self._check_stale)
        self._watch.start()

    def on_attitude(self, msg):
        roll = float(msg.get("roll", 0.0))
        pitch = float(msg.get("pitch", 0.0))
        yaw = float(msg.get("yaw", 0.0))
        rate = float(msg.get("yaw_rate", 0.0))
        self._last_rx = time.monotonic()
        self.horizon.stale = self.dial.stale = False
        self.horizon.set_attitude(roll, pitch)
        self.dial.set_heading(yaw, rate)
        self._values.setText(
            f"roll {roll:+.1f}\N{DEGREE SIGN}   pitch {pitch:+.1f}"
            f"\N{DEGREE SIGN}   hdg {yaw:03.0f}\N{DEGREE SIGN}")

    def _check_stale(self):
        if not self.horizon.stale and time.monotonic() - self._last_rx > 1.5:
            self.horizon.stale = self.dial.stale = True
            self._values.setText("—")
            self.horizon.update()
            self.dial.update()
