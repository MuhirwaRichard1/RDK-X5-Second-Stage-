"""AgentApp — glue between the WS server, the ROS bridge, and the launch
manager. Owns operator-facing state (mode, E-stop intent, log ring) and the
periodic telemetry broadcast. All methods run on the asyncio loop thread."""

import asyncio
import collections
import logging
import math
import time

from . import __version__, config, protocol
from .health import HealthSampler

log = logging.getLogger("navbot.app")


class AgentApp:
    def __init__(self, bridge=None, launch_mgr=None):
        self.hub = None                    # set by main() (server.Hub)
        self.bridge = bridge               # ros_bridge.RosBridge or None
        self.launch_mgr = launch_mgr       # launch_manager.LaunchManager or None
        self.video_pump = None             # video.VideoPump or None (set by main)
        self.map_pump = None               # map_pump.MapPump or None (set by main)
        self.health = HealthSampler()
        self.log_ring = collections.deque(maxlen=config.LOG_RING)

        self.estop_latched = False         # operator intent, survives relaunches
        self.mode = "stopped"
        self.mode_status = "active"
        self.mode_detail = ""
        self.udp_port = None               # set by main() when the UDP socket binds
        self.active_models = dict(config.MODEL_DEFAULTS)

    # ---------------- state / welcome ----------------

    def state_snapshot(self):
        mode_def = config.MODES.get(self.mode)
        return {
            "mode": self.mode,
            "mode_status": self.mode_status,
            "motors": bool(mode_def and mode_def[1]
                           and not config.FORCE_MOTORS_OFF),
            "estop": {"latched": self.estop_latched,
                      "confirmed": self.bridge.estop_confirmed if self.bridge else None},
            "detail": self.mode_detail,
            "models": dict(self.active_models),
        }

    def make_welcome(self, session=None):
        msg = protocol.welcome(
            __version__,
            {"v_max": config.V_MAX, "w_max": config.W_MAX},
            sorted(config.CAMERAS, key=config.CAMERAS.get),
            self.state_snapshot())
        if session is not None and self.udp_port:
            msg["udp"] = {"port": self.udp_port, "token": session.token.hex()}
        return msg

    def broadcast_state(self):
        if self.hub:
            self.hub.broadcast(protocol.state_msg(self.state_snapshot()))

    def set_mode_state(self, mode, status, detail=""):
        """Called by the launch manager on every transition step."""
        self.mode, self.mode_status, self.mode_detail = mode, status, detail
        if status == "active" and self.bridge:
            # the perception nodes just (re)started — their enable flags
            # reset to False, so re-assert whatever the operator had on.
            self.bridge.reassert_models()
        self.broadcast_state()

    # ---------------- logging ----------------

    def add_log(self, src, level, line):
        entry = protocol.log_msg(src, level, line)
        self.log_ring.append(entry)
        if self.hub:
            self.hub.broadcast(entry)

    # ---------------- client commands ----------------

    def on_teleop(self, vx, wz):
        try:
            vx = max(-config.V_MAX, min(config.V_MAX, float(vx)))
            wz = max(-config.W_MAX, min(config.W_MAX, float(wz)))
        except (TypeError, ValueError):
            return
        if self.bridge:
            self.bridge.set_teleop(vx, wz)

    def on_estop(self, engage):
        # Clients send E-stop on UDP *and* WS for reliability — log/broadcast
        # only on change, but always re-assert toward the bridge.
        changed = engage != self.estop_latched
        self.estop_latched = engage
        if changed:
            self.add_log("agent", "warn" if engage else "info",
                         f"E-stop {'ENGAGED' if engage else 'released'} by operator")
        if self.bridge:
            self.bridge.request_estop(engage)
        if changed:
            self.broadcast_state()

    def on_set_mode(self, mode, session):
        if mode not in config.MODES:
            session.send_json(protocol.error(f"unknown mode: {mode}"))
            return
        if not self.launch_mgr:
            session.send_json(protocol.error("mode control unavailable (--no-ros)"))
            return
        asyncio.get_running_loop().create_task(self.launch_mgr.set_mode(mode))

    def on_set_model(self, model, enable, session):
        if model not in config.MODELS:
            session.send_json(protocol.error(f"unknown model: {model}"))
            return
        if self.active_models[model] == enable:
            return
        self.active_models[model] = enable
        if self.bridge:
            self.bridge.set_model_enable(model, enable)
        self.broadcast_state()

    def on_video(self, session, msg):
        cam = msg.get("cam")
        if cam not in config.CAMERAS:
            session.send_json(protocol.error(f"unknown camera: {cam}"))
            return
        if msg.get("enable", True):
            session.video_cams.add(cam)
        else:
            session.video_cams.discard(cam)
        if self.video_pump:
            self.video_pump.configure(cam, fps=msg.get("fps"),
                                      quality=msg.get("quality"))

    def on_set_map(self, session, enable):
        session.wants_map = bool(enable)

    def on_save_map(self, session, name):
        """Save the live SLAM map to disk (both viewable .pgm/.yaml and
        loadable .posegraph/.data). Requires slam_toolbox running (a mapping
        mode). Result is reported back via the operator log."""
        if not self.bridge:
            session.send_json(protocol.error("map save unavailable (--no-ros)"))
            return
        base = str(name).strip() if name else config.DEFAULT_MAP
        # keep it a basename — never let a client write outside MAP_DIR
        base = base.replace("/", "_").replace("..", "_") or config.DEFAULT_MAP
        self.add_log("agent", "info", f"saving map -> {base}")
        self.bridge.save_map(base)

    def on_set_goal(self, session, x, y):
        """Publish a navigation goal (map frame) for goal_navigator."""
        if not self.bridge:
            session.send_json(protocol.error("goals unavailable (--no-ros)"))
            return
        try:
            x, y = float(x), float(y)
        except (TypeError, ValueError):
            session.send_json(protocol.error("goal needs numeric x, y"))
            return
        self.add_log("agent", "info", f"goal -> ({x:.2f}, {y:.2f}) m")
        self.bridge.set_goal(x, y)

    def on_client_gone(self, session):
        # Belt and braces: if the last teleop-capable client vanishes the
        # bridge staleness timer already zeros /cmd_vel within 400 ms.
        pass

    # ---------------- periodic telemetry ----------------

    async def telemetry_loop(self):
        period = 1.0 / config.TELEMETRY_HZ
        while True:
            await asyncio.sleep(period)
            if not self.hub or not self.hub.sessions:
                continue
            rates = self.bridge.take_rates() if self.bridge else {}
            range_cm = self.bridge.range_cm if self.bridge else None
            if range_cm is not None and math.isnan(range_cm):
                range_cm = None
            age = self.bridge.teleop_age_ms() if self.bridge else None
            odom = ({"source": self.bridge.odom_source(),
                     "pose_age_ms": self.bridge.pose_age_ms()}
                    if self.bridge else None)
            self.hub.broadcast_fast(protocol.telemetry(
                rates, range_cm, self.health.sample(), age, odom))

    async def attitude_loop(self):
        period = 1.0 / config.ATTITUDE_HZ
        while True:
            await asyncio.sleep(period)
            if not self.hub or not self.hub.sessions or not self.bridge:
                continue
            att = self.bridge.attitude
            if att is None or time.monotonic() - att[4] > 1.0:
                continue                   # no IMU running — send nothing
            self.hub.broadcast_fast(protocol.attitude(
                round(att[0], 1), round(att[1], 1),
                round(att[2], 1), round(att[3], 1)))
