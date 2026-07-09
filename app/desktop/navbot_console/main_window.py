"""MainWindow — layout and wiring.

Left: video (front + sides). Right: mode bar, E-stop, joystick + speed,
health. Bottom: log. Keyboard: WASD/arrows drive (manual mode), Space
engages the E-stop from anywhere except text fields."""

from PySide6.QtCore import QEvent, QSettings, Qt
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                               QPushButton, QScrollArea, QSlider, QSplitter,
                               QVBoxLayout, QWidget)

from .client import RobotClient
from .teleop import TeleopController
from .widgets.attitude_panel import AttitudePanel
from .widgets.estop_button import EStopButton
from .widgets.health_panel import HealthPanel
from .widgets.joystick import JoystickWidget
from .widgets.log_panel import LogPanel
from .widgets.mode_bar import ModeBar
from .widgets.model_bar import ModelBar
from .widgets.video_panel import VideoPanel

_KEYMAP = {Qt.Key_W: "fwd", Qt.Key_Up: "fwd",
           Qt.Key_S: "back", Qt.Key_Down: "back",
           Qt.Key_A: "left", Qt.Key_Left: "left",
           Qt.Key_D: "right", Qt.Key_Right: "right"}

_ACTIVATE_STYLE = {
    "idle":   "font-weight:bold;font-size:14px;padding:8px;",
    "busy":   "font-weight:bold;font-size:14px;padding:8px;background:#444;color:#aaa;",
    "active": "font-weight:bold;font-size:14px;padding:8px;background:#00c853;color:black;",
}
_AVOID_LAMP = {
    True:  "background:#00c853;color:black;border-radius:9px;padding:4px 10px;font-weight:bold;",
    False: "background:#c62828;color:white;border-radius:9px;padding:4px 10px;font-weight:bold;",
}


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NavBot Console")
        self.resize(1400, 850)
        self._settings = QSettings("navbot", "console")
        self._state = {}
        self._pending_activate = False

        self.client = RobotClient(self)
        self.teleop = TeleopController(self.client, self)

        # ---- widgets ----
        self.video = VideoPanel()
        self._activate_btn = QPushButton("Activate Robot")
        self._activate_btn.setMinimumHeight(44)
        self._activate_btn.setStyleSheet(_ACTIVATE_STYLE["idle"])
        self._avoid_lamp = QLabel("OBSTACLE AVOIDANCE OFF")
        self._avoid_lamp.setStyleSheet(_AVOID_LAMP[False])
        self.mode_bar = ModeBar()
        self.model_bar = ModelBar()
        self.estop = EStopButton()
        self.joystick = JoystickWidget()
        self.attitude = AttitudePanel()
        self.health = HealthPanel()
        self.log = LogPanel()

        self.speed = QSlider(Qt.Horizontal, minimum=10, maximum=100, value=50)
        self._speed_label = QLabel("speed 50%")
        self.speed.valueChanged.connect(self._on_speed)

        self._url_edit = QLineEdit(
            self._settings.value("robot_url", "ws://192.168.1.76:8080"))
        self._connect_btn = QPushButton("Connect")
        self._conn_dot = QLabel("●")
        self._conn_dot.setStyleSheet("color:#888;font-size:16px;")
        self._teleop_readout = QLabel("vx 0.00  wz 0.00")

        # ---- layout ----
        top = QHBoxLayout()
        top.addWidget(QLabel("robot:"))
        top.addWidget(self._url_edit, 1)
        top.addWidget(self._connect_btn)
        top.addWidget(self._conn_dot)

        right = QVBoxLayout()
        right.addWidget(self._activate_btn)
        right.addWidget(self._avoid_lamp)
        right.addWidget(self.mode_bar)
        right.addWidget(self.model_bar)
        right.addWidget(self.estop)
        right.addWidget(self.joystick, 0, Qt.AlignHCenter)
        speed_row = QHBoxLayout()
        speed_row.addWidget(self._speed_label)
        speed_row.addWidget(self.speed, 1)
        right.addLayout(speed_row)
        right.addWidget(self._teleop_readout)
        right.addWidget(self.attitude)
        right.addWidget(self.health, 1)
        right_w = QWidget()
        right_w.setLayout(right)
        right_w.setMaximumWidth(360)

        right_scroll = QScrollArea()
        right_scroll.setWidget(right_w)
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setMaximumWidth(380)
        right_scroll.setMinimumWidth(280)

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.video)
        split.addWidget(right_scroll)
        split.setStretchFactor(0, 1)

        vsplit = QSplitter(Qt.Vertical)
        vsplit.addWidget(split)
        vsplit.addWidget(self.log)
        vsplit.setStretchFactor(0, 4)
        vsplit.setStretchFactor(1, 1)

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addLayout(top)
        lay.addWidget(vsplit, 1)
        self.setCentralWidget(central)

        # ---- wiring ----
        self._activate_btn.clicked.connect(self._on_activate_clicked)
        self._connect_btn.clicked.connect(self._toggle_connection)
        self.client.connectedChanged.connect(self._on_connected)
        self.client.welcomeReceived.connect(self._on_welcome)
        self.client.stateReceived.connect(self._on_state)
        self.client.telemetryReceived.connect(self.health.on_telemetry)
        self.client.latencyMs.connect(self.health.on_latency)
        self.client.transportChanged.connect(self.health.on_transport)
        self.client.sectorsReceived.connect(self.video.on_sectors)
        self.client.gridOverlayReceived.connect(self.video.on_grid_overlay)
        self.client.detectionsReceived.connect(self.video.on_detections)
        self.client.attitudeReceived.connect(self.attitude.on_attitude)
        self.client.logReceived.connect(self.log.on_log)
        self.client.errorReceived.connect(
            lambda m: self.log.on_log({"src": "agent", "level": "error", "line": m}))
        self.client.videoFrame.connect(self.video.on_frame)
        self.client.staleChanged.connect(self._on_stale)
        self.video.cameraToggled.connect(
            lambda cam, on: self.client.send_video(cam, on))
        self.mode_bar.modeRequested.connect(self.client.send_mode)
        self.model_bar.modelToggled.connect(self._on_model_toggled)
        self.estop.estopRequested.connect(self.client.send_estop)
        self.joystick.moved.connect(self.teleop.joystick)
        self.teleop.commandChanged.connect(
            lambda vx, wz: self._teleop_readout.setText(f"vx {vx:+.2f}  wz {wz:+.2f}"))
        self._on_speed(self.speed.value())
        self._update_activate_btn()

    # ---------------- activate ----------------

    def _on_activate_clicked(self):
        if self._state.get("mode") == "manual" and self._state.get("mode_status") == "active":
            return                       # already active; nothing to do
        if self._state.get("mode_status") in ("starting", "stopping"):
            return                       # transition already in flight
        if not self.client.connected:
            self._pending_activate = True
            self._toggle_connection()
        else:
            self.client.send_mode("manual")
        self._update_activate_btn()

    def _update_activate_btn(self):
        if self._pending_activate:
            self._activate_btn.setEnabled(False)
            self._activate_btn.setText("Activating…")
            self._activate_btn.setStyleSheet(_ACTIVATE_STYLE["busy"])
            return
        mode = self._state.get("mode")
        status = self._state.get("mode_status")
        if mode == "manual" and status == "active":
            self._activate_btn.setEnabled(True)
            self._activate_btn.setText("Robot Active")
            self._activate_btn.setStyleSheet(_ACTIVATE_STYLE["active"])
        elif status in ("starting", "stopping"):
            self._activate_btn.setEnabled(False)
            self._activate_btn.setText("Activating…")
            self._activate_btn.setStyleSheet(_ACTIVATE_STYLE["busy"])
        else:
            self._activate_btn.setEnabled(True)
            self._activate_btn.setText("Activate Robot")
            self._activate_btn.setStyleSheet(_ACTIVATE_STYLE["idle"])

    def _on_model_toggled(self, model, enabled):
        self.client.send_model(model, enabled)
        self.video.set_model_overlay_enabled(model, enabled)

    # ---------------- connection ----------------

    def _toggle_connection(self):
        if self.client.connected:
            self.client.close()
        else:
            url = self._url_edit.text().strip()
            if not url.startswith("ws://"):
                url = "ws://" + url
                self._url_edit.setText(url)
            self._settings.setValue("robot_url", url)
            self.client.open(url)
            self._connect_btn.setText("Connecting…")

    def _on_connected(self, ok):
        self._connect_btn.setText("Disconnect" if ok else "Connect")
        self._conn_dot.setStyleSheet(
            f"color:{'#00c853' if ok else '#888'};font-size:16px;")
        self.mode_bar.set_connected(ok)
        self.model_bar.set_connected(ok)
        if ok:
            for cam in self.video.enabled_cams():
                self.client.send_video(cam, True)
        else:
            self.video.clear_all()
            self.teleop.set_enabled(False)
            self._pending_activate = False
            self._state = {}
            self._avoid_lamp.setText("OBSTACLE AVOIDANCE OFF")
            self._avoid_lamp.setStyleSheet(_AVOID_LAMP[False])
            self._update_activate_btn()

    def _on_stale(self, stale):
        self._conn_dot.setStyleSheet(
            f"color:{'#ffd166' if stale else '#00c853'};font-size:16px;")
        if stale:
            self.teleop.set_enabled(False)

    # ---------------- robot state ----------------

    def _on_welcome(self, msg):
        lim = msg.get("limits", {})
        self.teleop.set_limits(lim.get("v_max", 0.4), lim.get("w_max", 1.2))
        if self._pending_activate:
            self._pending_activate = False
            self.client.send_mode("manual")
            self._update_activate_btn()

    def _on_state(self, state):
        self._state = state
        self.mode_bar.set_state(state)
        self.model_bar.set_state(state)
        es = state.get("estop", {})
        self.estop.set_state(bool(es.get("latched")), es.get("confirmed"))
        driving = (state.get("mode") == "manual"
                   and state.get("mode_status") == "active"
                   and not es.get("latched"))
        self.teleop.set_enabled(driving)
        avoidance_on = bool(state.get("models", {}).get("obstacle_avoidance"))
        self._avoid_lamp.setText(
            "OBSTACLE AVOIDANCE ON" if avoidance_on else "OBSTACLE AVOIDANCE OFF")
        self._avoid_lamp.setStyleSheet(_AVOID_LAMP[avoidance_on])
        self._update_activate_btn()

    def _on_speed(self, v):
        self._speed_label.setText(f"speed {v}%")
        self.teleop.set_speed(v / 100.0)

    # ---------------- keyboard (app-wide) ----------------

    def eventFilter(self, _obj, ev):
        if ev.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            if isinstance(self.focusWidget(), QLineEdit):
                return False                 # let text fields type normally
            if ev.isAutoRepeat():
                return True
            key = ev.key()
            if key == Qt.Key_Space and ev.type() == QEvent.KeyPress:
                self.estop.engage()
                return True
            action = _KEYMAP.get(key)
            if action:
                self.teleop.key_event(action, ev.type() == QEvent.KeyPress)
                return True
        elif ev.type() == QEvent.WindowDeactivate:
            self.teleop.release_all()        # alt-tab must not keep driving
        return False
