"""ModelBar — yolo11/depthanything overlay toggles. depth is now the primary
obstacle sensor and always runs; its button toggles the depth HUD overlay
only. yolo11 lazy-loads on the BPU. Buttons disable while the mode stack
isn't fully up — the perception nodes that own these toggles only exist once
a mode is active. (The old PIDNET button was removed when depth_freespace
replaced PIDNet obstacle_fusion.)"""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QWidget

_MODELS = ("yolo11", "depthanything")
_STYLE = {True: "background:#00c853;color:black;font-weight:bold;",
         False: ""}


class ModelBar(QWidget):
    modelToggled = Signal(str, bool)      # model name, enabled

    def __init__(self, parent=None):
        super().__init__(parent)
        self._buttons = {}
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        for m in _MODELS:
            b = QPushButton(m.upper())
            b.setCheckable(True)
            b.setMinimumHeight(32)
            b.clicked.connect(lambda _c, name=m: self._on_click(name))
            row.addWidget(b)
            self._buttons[m] = b
        self.set_connected(False)

    def _on_click(self, name):
        enabled = self._buttons[name].isChecked()
        self._buttons[name].setStyleSheet(_STYLE[enabled])
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
            b.setStyleSheet(_STYLE[on])
            b.setEnabled(active)

    def set_connected(self, ok):
        for b in self._buttons.values():
            b.setEnabled(ok)
            if not ok:
                b.setChecked(False)
                b.setStyleSheet(_STYLE[False])
