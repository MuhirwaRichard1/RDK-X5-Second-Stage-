"""TeleopController — merges keyboard (WASD/arrows) and joystick input into
20 Hz teleop messages. On release it sends a short burst of zeros and goes
silent; the agent's 400 ms staleness rule and motor_controller's 200 ms
dead-man are the safety net beneath that."""

import time

from PySide6.QtCore import QObject, QTimer, Signal


class TeleopController(QObject):
    commandChanged = Signal(float, float)    # vx, wz — for the UI readout

    def __init__(self, client, parent=None):
        super().__init__(parent)
        self._client = client
        self._keys = set()                   # {"fwd","back","left","right"}
        self._joy = (0.0, 0.0)               # normalized (wz, vx), joystick wins
        self._joy_active = False
        self._speed = 0.5                    # slider 0..1
        self._v_max = 0.40
        self._w_max = 1.2
        self._enabled = False
        self._zeros_left = 0
        self._seq = 0

        self._timer = QTimer(self, interval=50)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # ---------------- inputs ----------------

    def set_limits(self, v_max, w_max):
        self._v_max, self._w_max = v_max, w_max

    def set_speed(self, frac):
        self._speed = max(0.0, min(1.0, frac))

    def set_enabled(self, enabled):
        if self._enabled and not enabled:
            self._zeros_left = 4             # stop cleanly on mode change
        self._enabled = enabled

    def key_event(self, action, pressed):
        if pressed:
            self._keys.add(action)
        else:
            self._keys.discard(action)

    def joystick(self, x, y):
        """x right+, y up+ — both -1..1; (0,0) = released."""
        self._joy = (x, y)
        self._joy_active = abs(x) > 0.05 or abs(y) > 0.05

    def release_all(self):
        self._keys.clear()
        self._joy = (0.0, 0.0)
        self._joy_active = False

    # ---------------- 20 Hz tick ----------------

    def _command(self):
        if self._joy_active:
            jx, jy = self._joy
            return jy * self._v_max * self._speed, -jx * self._w_max
        vx = wz = 0.0
        if "fwd" in self._keys:
            vx += 1.0
        if "back" in self._keys:
            vx -= 1.0
        if "left" in self._keys:
            wz += 1.0                        # REP-103: +wz = CCW = left
        if "right" in self._keys:
            wz -= 1.0
        return vx * self._v_max * self._speed, wz * self._w_max * 0.7

    def _tick(self):
        vx, wz = self._command() if self._enabled else (0.0, 0.0)
        active = self._enabled and (vx or wz)
        if active:
            self._zeros_left = 4
        elif self._zeros_left > 0:
            self._zeros_left -= 1
            vx = wz = 0.0
        else:
            return                           # silent while idle
        self._seq += 1
        self._client.send_teleop(round(vx, 3), round(wz, 3), self._seq)
        self.commandChanged.emit(vx, wz)
