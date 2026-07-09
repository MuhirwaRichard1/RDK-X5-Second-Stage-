"""ModelBar — operator-togglable models/features. obstacle_avoidance gates
safety_gate's visual (depth-derived — depth_freespace replaced PIDNet
obstacle_fusion as the /obstacles publisher) forward-block and is a safety
feature (off is drawn as a warning, not just "unchecked"). yolo11 and
depthanything are perception overlays, independent of each other and of
obstacle_avoidance: yolo11 lazy-loads on the BPU, depthanything toggles the
depth HUD overlay only (depth_freespace itself always runs). Buttons
disable while the mode stack isn't fully up — the nodes that own these
toggles only exist once a mode is active."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QWidget

_MODELS = ("obstacle_avoidance", "yolo11", "depthanything")
_LABELS = {"obstacle_avoidance": "OBSTACLE AVOIDANCE",
          "yolo11": "YOLO11",
          "depthanything": "DEPTHANYTHING"}
_STYLE_ON = "background:#00c853;color:black;font-weight:bold;"
_STYLE_OFF = ""
_STYLE_WARN_OFF = "background:#c62828;color:white;font-weight:bold;"


def _style(model, on):
    if on:
        return _STYLE_ON
    return _STYLE_WARN_OFF if model == "obstacle_avoidance" else _STYLE_OFF


class ModelBar(QWidget):
    modelToggled = Signal(str, bool)      # model name, enabled

    def __init__(self, parent=None):
        super().__init__(parent)
        self._buttons = {}
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        for m in _MODELS:
            b = QPushButton(_LABELS[m])
            b.setCheckable(True)
            b.setMinimumHeight(32)
            b.clicked.connect(lambda _c, name=m: self._on_click(name))
            row.addWidget(b)
            self._buttons[m] = b
        self.set_connected(False)

    def _on_click(self, name):
        enabled = self._buttons[name].isChecked()
        self._buttons[name].setStyleSheet(_style(name, enabled))
        self.modelToggled.emit(name, enabled)

    def set_state(self, state: dict):
        """Authoritative refresh from the agent's state broadcast — wins
        over any optimistic click, mirrors ModeBar.set_state."""
        models = state.get("models") or {}
        active = state.get("mode") not in (None, "stopped") \
            and state.get("mode_status") == "active"
        for m, b in self._buttons.items():
            on = bool(models.get(m))
            b.setChecked(on)
            b.setStyleSheet(_style(m, on))
            b.setEnabled(active)

    def set_connected(self, ok):
        for b in self._buttons.values():
            b.setEnabled(ok)
            if not ok:
                b.setChecked(False)
                b.setStyleSheet(_STYLE_OFF)
