"""LogPanel — launch stdout + agent events, color-coded, bounded."""

import html

from PySide6.QtWidgets import QPlainTextEdit

_COLOR = {"error": "#ff6b6b", "warn": "#ffd166", "info": "#c8c8c8"}


class LogPanel(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(2000)
        self.setStyleSheet("font-family: monospace; font-size: 11px;"
                           "background: #101010; color: #c8c8c8;")

    def on_log(self, entry: dict):
        color = _COLOR.get(entry.get("level"), "#c8c8c8")
        src = entry.get("src", "?")
        line = html.escape(entry.get("line", ""))
        self.appendHtml(f'<span style="color:{color}">[{src}] {line}</span>')
