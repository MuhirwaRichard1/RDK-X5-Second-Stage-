"""HealthPanel — live robot vitals from the 2 Hz telemetry message."""

from PySide6.QtWidgets import (QFormLayout, QGroupBox, QLabel, QProgressBar,
                               QVBoxLayout, QWidget)

_RATE_ROWS = [("/cam_front/image_raw", "front cam", 25.0),
              ("/cam_left/image_raw", "left cam", 10.0),
              ("/cam_right/image_raw", "right cam", 10.0),
              ("/obstacles", "obstacles", 8.0),
              ("/cmd_vel", "cmd_vel", 1.0),
              ("/cmd_vel_safe", "cmd_vel_safe", 1.0),
              ("/range_forward", "lidar range", 15.0)]


def _bar():
    b = QProgressBar()
    b.setRange(0, 100)
    b.setTextVisible(True)
    b.setMaximumHeight(16)
    return b


class HealthPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._range = QLabel("—")
        self._latency = QLabel("—")
        self._transport = "TCP"
        self._teleop_age = QLabel("—")
        self._cpu, self._mem, self._bpu = _bar(), _bar(), _bar()
        self._temps = QLabel("—")
        self._wifi = QLabel("—")
        self._rates = {}

        vitals = QGroupBox("robot vitals")
        f = QFormLayout(vitals)
        f.addRow("forward range", self._range)
        f.addRow("link RTT", self._latency)
        f.addRow("teleop age", self._teleop_age)
        f.addRow("CPU", self._cpu)
        f.addRow("memory", self._mem)
        f.addRow("BPU", self._bpu)
        f.addRow("temps", self._temps)
        f.addRow("WiFi", self._wifi)

        topics = QGroupBox("topic rates (Hz)")
        tf = QFormLayout(topics)
        for topic, label, _min in _RATE_ROWS:
            lab = QLabel("—")
            self._rates[topic] = lab
            tf.addRow(label, lab)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(vitals)
        lay.addWidget(topics)

    def on_telemetry(self, t: dict):
        r = t.get("range_cm")
        if r is None:
            self._range.setText("no reading")
            self._range.setStyleSheet("color:#ff5555;font-weight:bold;")
        else:
            self._range.setText(f"{r:.0f} cm")
            self._range.setStyleSheet(
                "color:#ff5555;font-weight:bold;" if r < 30 else "")
        age = t.get("teleop_age_ms")
        self._teleop_age.setText(
            f"{age} ms" if age is not None and age < 5000 else "—")
        self._cpu.setValue(int(t.get("cpu") or 0))
        self._mem.setValue(int(t.get("mem") or 0))
        self._bpu.setValue(int(t.get("bpu_pct") or 0))
        temps = [f"{k.split('_')[1]} {v:.0f}°C" for k, v in t.items()
                 if k.startswith("temp_") and v is not None]
        self._temps.setText("  ".join(temps) or "—")
        wifi = t.get("wifi_dbm")
        self._wifi.setText(f"{wifi} dBm" if wifi is not None else "—")
        rates = t.get("rates", {})
        for topic, label, min_ok in _RATE_ROWS:
            v = rates.get(topic)
            lab = self._rates[topic]
            lab.setText(f"{v:.1f}" if v is not None else "—")
            lab.setStyleSheet("" if (v or 0) >= min_ok else "color:#999;")

    def on_latency(self, ms):
        self._latency.setText(f"{ms:.0f} ms · {self._transport}")

    def on_transport(self, name):
        self._transport = name
