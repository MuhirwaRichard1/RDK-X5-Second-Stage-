"""LaunchManager — owns the single ros2 launch subprocess behind the
operator's mode (stopped / observe / manual / auto).

Robustness rules:
  * The child runs in its own process group; stop = SIGINT to the group
    (ros2 launch shuts nodes down cleanly), then SIGTERM, then SIGKILL.
  * The group id is written to a pidfile so a crashed-and-restarted agent
    kills any orphan before doing anything else.
  * A watchdog notices the child dying on its own (node crash, OOM) and
    reports mode=stopped status=error to the operator.
  * All transitions are serialized by one asyncio.Lock."""

import asyncio
import logging
import os
import signal
import time

from . import config

log = logging.getLogger("navbot.launch")


class LaunchManager:
    def __init__(self, bridge):
        self.bridge = bridge
        self.app = None
        self.proc = None
        self.mode = "stopped"
        self._lock = asyncio.Lock()
        self._log_task = None
        self._pidfile = self._pick_pidfile()

    @staticmethod
    def _pick_pidfile():
        try:
            with open(config.LAUNCH_PIDFILE, "a"):
                return config.LAUNCH_PIDFILE
        except OSError:
            return "/tmp/navbot-agent.launch.pid"

    def attach(self, app):
        self.app = app

    # ---------------- stale-orphan cleanup ----------------

    def kill_stale_launch(self):
        try:
            with open(self._pidfile) as f:
                pgid = int(f.read().strip())
        except (OSError, ValueError):
            return
        try:
            os.killpg(pgid, signal.SIGKILL)
            log.warning("killed stale launch process group %d", pgid)
        except ProcessLookupError:
            pass
        except PermissionError:
            log.error("stale launch pgid %d exists but not killable", pgid)
        self._clear_pidfile()

    def _clear_pidfile(self):
        try:
            os.remove(self._pidfile)
        except OSError:
            pass

    # ---------------- transitions ----------------

    async def set_mode(self, mode):
        async with self._lock:
            if mode == self.mode and self._alive():
                return
            await self._stop_current()
            if mode == "stopped":
                self.mode = "stopped"
                self.app.set_mode_state("stopped", "active")
                return
            await self._start(mode)

    async def shutdown(self):
        async with self._lock:
            await self._stop_current()
            self.mode = "stopped"

    def _alive(self):
        return self.proc is not None and self.proc.returncode is None

    async def _start(self, mode):
        launch_file, motors = config.MODES[mode]
        motors = motors and not config.FORCE_MOTORS_OFF
        self.mode = mode
        self.app.set_mode_state(mode, "starting", f"launching {launch_file}")
        cmd = (f"source {config.ROS_SETUP} && source {config.WS_SETUP} && "
               f"exec ros2 launch {config.LAUNCH_PKG} {launch_file} "
               f"motors:={'true' if motors else 'false'}")
        self.proc = await asyncio.create_subprocess_exec(
            "bash", "-c", cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True, cwd=config.WS_ROOT)
        with open(self._pidfile, "w") as f:
            f.write(str(self.proc.pid))            # pid == pgid (new session)
        self._log_task = asyncio.create_task(self._pump_logs(self.proc))

        # Teleop only in manual mode; the E-stop reconciler re-asserts the
        # operator latch on the fresh safety_gate automatically.
        self.bridge.teleop_enabled = (mode == "manual")

        if await self._wait_active():
            self.app.set_mode_state(mode, "active")
        else:
            self.app.set_mode_state(mode, "error", "startup timeout — stopping")
            await self._stop_current()
            self.mode = "stopped"
            self.app.set_mode_state("stopped", "error",
                                    "mode failed to start (see log)")

    async def _wait_active(self):
        """Active = /obstacles is flowing (every mode runs obstacle_fusion)."""
        n0 = self.bridge._counters["/obstacles"]
        deadline = time.monotonic() + config.MODE_ACTIVE_TIMEOUT_S
        while time.monotonic() < deadline:
            if not self._alive():
                return False
            if self.bridge._counters["/obstacles"] > n0 + 3:
                return True
            await asyncio.sleep(0.5)
        return False

    async def _stop_current(self):
        self.bridge.teleop_enabled = False
        proc, self.proc = self.proc, None
        if self._log_task:
            self._log_task.cancel()
            self._log_task = None
        if proc is None or proc.returncode is not None:
            self._clear_pidfile()
            return
        self.app.set_mode_state(self.mode, "stopping")
        for sig, grace in ((signal.SIGINT, config.LAUNCH_STOP_SIGINT_S),
                           (signal.SIGTERM, config.LAUNCH_STOP_SIGTERM_S),
                           (signal.SIGKILL, 5.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                await asyncio.wait_for(proc.wait(), timeout=grace)
                break
            except asyncio.TimeoutError:
                log.warning("launch ignored %s, escalating", sig.name)
        self._clear_pidfile()
        self.app.add_log("agent", "info", "launch stopped")

    # ---------------- child monitoring ----------------

    async def _pump_logs(self, proc):
        try:
            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                text = line.decode(errors="replace").rstrip()
                if text:
                    level = ("error" if "[ERROR]" in text
                             else "warn" if "[WARN]" in text else "info")
                    self.app.add_log("launch", level, text)
        except asyncio.CancelledError:
            pass

    async def watchdog(self):
        """Detect the launch dying on its own."""
        while True:
            await asyncio.sleep(1.0)
            if self.mode == "stopped" or self._lock.locked():
                continue
            if self.proc is not None and self.proc.returncode is not None:
                async with self._lock:
                    if self.proc is None or self.proc.returncode is None:
                        continue                    # a transition beat us
                    rc = self.proc.returncode
                    pgid = self.proc.pid
                    self.proc = None
                    self.bridge.teleop_enabled = False
                    dead_mode, self.mode = self.mode, "stopped"
                    self.app.set_mode_state(
                        "stopped", "error",
                        f"{dead_mode} launch exited unexpectedly (rc={rc})")
                    # ros2 launch died without cleaning up (e.g. SIGKILL):
                    # its nodes keep the group id — sweep them.
                    try:
                        os.killpg(pgid, signal.SIGINT)
                        await asyncio.sleep(5.0)
                        os.killpg(pgid, signal.SIGKILL)
                        self.app.add_log("agent", "warn",
                                         "swept orphaned launch nodes")
                    except ProcessLookupError:
                        pass
                    self._clear_pidfile()
