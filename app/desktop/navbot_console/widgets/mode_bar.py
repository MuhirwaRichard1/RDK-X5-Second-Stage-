"""ModeBar — the four drive modes as exclusive buttons + a motors lamp.
Buttons disable during starting/stopping so transitions can't stack."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QButtonGroup, QHBoxLayout, QLabel,
                               QPushButton, QVBoxLayout, QWidget)

_MODES = ("stopped", "observe", "manual", "auto", "mapping", "navigate")
_LAMP = {True: "background:#00c853;color:black;border-radius:9px;padding:4px 10px;font-weight:bold;",
         False: "background:#444;color:#aaa;border-radius:9px;padding:4px 10px;"}


class ModeBar(QWidget):
    modeRequested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        buttons_row = QHBoxLayout()
        self._group = QButtonGroup(self, exclusive=True)
        self._buttons = {}
        for m in _MODES:
            b = QPushButton(m.upper())
            b.setCheckable(True)
            b.setMinimumHeight(36)
            b.clicked.connect(lambda _c, mode=m: self.modeRequested.emit(mode))
            self._group.addButton(b)
            self._buttons[m] = b
            buttons_row.addWidget(b)
        self._buttons["stopped"].setChecked(True)

        self._status = QLabel("")
        self._lamp = QLabel("MOTORS OFF")
        self._lamp.setStyleSheet(_LAMP[False])
        status_row = QHBoxLayout()
        status_row.addWidget(self._status, 1)
        status_row.addWidget(self._lamp)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(buttons_row)
        lay.addLayout(status_row)

    def set_state(self, state: dict):
        mode = state.get("mode", "stopped")
        status = state.get("mode_status", "")
        if mode in self._buttons:
            self._buttons[mode].setChecked(True)
        busy = status in ("starting", "stopping")
        for b in self._buttons.values():
            b.setEnabled(not busy)
        txt = status if status != "active" else ""
        detail = state.get("detail") or ""
        self._status.setText(f"{txt} {detail}".strip())
        motors = bool(state.get("motors")) and status == "active"
        self._lamp.setText("MOTORS ON" if motors else "MOTORS OFF")
        self._lamp.setStyleSheet(_LAMP[motors])

    def set_connected(self, ok):
        for b in self._buttons.values():
            b.setEnabled(ok)
