"""ModeBar — the drive modes as exclusive buttons + a motors lamp. Laid out
in a wrapping grid so all six fit (and stay readable) in the narrow control
column. Buttons disable during starting/stopping so transitions can't stack."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QButtonGroup, QGridLayout, QHBoxLayout, QLabel,
                               QPushButton, QSizePolicy, QVBoxLayout, QWidget)

_MODES = ("stopped", "observe", "manual", "auto", "mapping", "navigate")
_COLS = 3       # 6 modes -> two rows of three; fits the ~360 px column
_LAMP = {True: "background:#00c853;color:black;border-radius:9px;padding:4px 10px;font-weight:bold;",
         False: "background:#444;color:#aaa;border-radius:9px;padding:4px 10px;"}


class ModeBar(QWidget):
    modeRequested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        buttons_grid = QGridLayout()
        buttons_grid.setContentsMargins(0, 0, 0, 0)
        buttons_grid.setSpacing(4)
        self._group = QButtonGroup(self, exclusive=True)
        self._buttons = {}
        for i, m in enumerate(_MODES):
            b = QPushButton(m.upper())
            b.setCheckable(True)
            b.setMinimumHeight(36)
            b.setMinimumWidth(0)               # let it shrink to the cell
            b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            b.clicked.connect(lambda _c, mode=m: self.modeRequested.emit(mode))
            self._group.addButton(b)
            self._buttons[m] = b
            buttons_grid.addWidget(b, i // _COLS, i % _COLS)
        for c in range(_COLS):
            buttons_grid.setColumnStretch(c, 1)   # equal-width columns
        self._buttons["stopped"].setChecked(True)

        self._status = QLabel("")
        self._lamp = QLabel("MOTORS OFF")
        self._lamp.setStyleSheet(_LAMP[False])
        status_row = QHBoxLayout()
        status_row.addWidget(self._status, 1)
        status_row.addWidget(self._lamp)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(buttons_grid)
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
