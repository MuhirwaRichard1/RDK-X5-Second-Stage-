"""EStopButton — oversized kill switch. One click (or Space anywhere in the
app) ENGAGES; releasing requires a deliberate second click on the button.
Shows agent latch vs safety_gate confirmation ("pending" while they differ)."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QPushButton

_STYLE_ARMED = """
QPushButton { background: #b30000; color: white; font-size: 22px;
              font-weight: bold; border-radius: 10px; padding: 18px; }
QPushButton:hover { background: #d40000; }"""
_STYLE_LATCHED = """
QPushButton { background: #4d0000; color: #ffb3b3; font-size: 18px;
              font-weight: bold; border-radius: 10px; padding: 18px;
              border: 3px solid #ff4444; }"""


class EStopButton(QPushButton):
    estopRequested = Signal(bool)            # True = engage

    def __init__(self, parent=None):
        super().__init__("E-STOP", parent)
        self.setMinimumHeight(100)
        self._latched = False
        self._confirmed = None
        self.setStyleSheet(_STYLE_ARMED)
        self.clicked.connect(self._on_click)

    def _on_click(self):
        self.estopRequested.emit(not self._latched)

    def engage(self):
        """Called from the global Space shortcut — engage only, never release."""
        if not self._latched:
            self.estopRequested.emit(True)

    def set_state(self, latched, confirmed):
        self._latched, self._confirmed = latched, confirmed
        if latched:
            suffix = "" if confirmed else "  (pending…)"
            self.setText(f"ENGAGED{suffix}\nclick to RELEASE")
            self.setStyleSheet(_STYLE_LATCHED)
        else:
            self.setText("E-STOP\n(Space)")
            self.setStyleSheet(_STYLE_ARMED)
