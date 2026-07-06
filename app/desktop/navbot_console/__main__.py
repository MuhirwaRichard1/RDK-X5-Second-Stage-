"""NavBot Console entry point.

    python -m navbot_console                   # normal GUI
    python -m navbot_console --url ws://192.168.1.76:8080
    python -m navbot_console --self-test       # headless CI check: connect to
                                               # a local agent, expect welcome
                                               # + telemetry, exit 0/1
"""

import argparse
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from .main_window import MainWindow


def main():
    p = argparse.ArgumentParser(prog="navbot_console")
    p.add_argument("--url", help="robot WebSocket URL (remembered)")
    p.add_argument("--self-test", action="store_true",
                   help="headless: connect to ws://127.0.0.1:8080, verify "
                        "welcome + telemetry arrive, exit")
    args = p.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow()
    app.installEventFilter(win)

    if args.self_test:
        got = {"welcome": False, "telemetry": False}
        win.client.welcomeReceived.connect(
            lambda _m: got.__setitem__("welcome", True))
        win.client.telemetryReceived.connect(
            lambda _m: (got.__setitem__("telemetry", True), app.exit(0)))
        win._url_edit.setText("ws://127.0.0.1:8080")
        win._toggle_connection()
        QTimer.singleShot(8000, lambda: app.exit(1))
        rc = app.exec()
        print(f"self-test: welcome={got['welcome']} telemetry={got['telemetry']}"
              f" -> {'PASS' if rc == 0 else 'FAIL'}")
        sys.exit(rc)

    if args.url:
        win._url_edit.setText(args.url)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
