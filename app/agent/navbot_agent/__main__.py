"""navbot agent entry point.

    sudo python3 -m navbot_agent            # full agent (ROS + launch control)
    python3 -m navbot_agent --no-ros        # WS server + health only (dev/test)

The agent owns everything an operator client needs: WebSocket API on
:8080, ros2 launch lifecycle for the drive modes, teleop relay to
/cmd_vel, the /estop latch, telemetry, and video forwarding."""

import argparse
import asyncio
import logging
import signal

from . import __version__, config
from .app import AgentApp
from .server import Hub, UdpServer, WsServer

log = logging.getLogger("navbot.main")


async def _amain(args):
    bridge = launch_mgr = video_pump = None
    if not args.no_ros:
        from .launch_manager import LaunchManager
        from .ros_bridge import RosBridge
        from .video import VideoPump
        bridge = RosBridge()
        launch_mgr = LaunchManager(bridge)
        video_pump = VideoPump(bridge)

    app = AgentApp(bridge=bridge, launch_mgr=launch_mgr)
    app.hub = Hub()
    app.video_pump = video_pump

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    udp_transport = None
    try:
        udp_transport, udp_server = await loop.create_datagram_endpoint(
            lambda: UdpServer(app), local_addr=(args.host, args.port))
        app.hub.udp = udp_server
        app.udp_port = args.port
        log.info("UDP fast path on udp://%s:%d", args.host, args.port)
    except OSError as e:
        log.warning("UDP fast path unavailable (%s) — WS only", e)

    tasks = [asyncio.create_task(WsServer(app, args.host, args.port).run()),
             asyncio.create_task(app.telemetry_loop())]
    try:
        if bridge:
            bridge.start(app, loop)
        if launch_mgr:
            launch_mgr.attach(app)
            launch_mgr.kill_stale_launch()
            tasks.append(asyncio.create_task(launch_mgr.watchdog()))
        if video_pump:
            video_pump.attach(app.hub)
            tasks += [asyncio.create_task(video_pump.pump(cam))
                      for cam in config.CAMERAS]

        app.add_log("agent", "info", f"navbot-agent {__version__} up"
                    + (" (--no-ros)" if args.no_ros else ""))
        await stop.wait()
        log.info("shutting down")
    finally:
        for t in tasks:
            t.cancel()
        if udp_transport:
            udp_transport.close()
        if launch_mgr:
            await launch_mgr.shutdown()        # never leave a launch orphaned
        if bridge:
            bridge.stop()


def main():
    p = argparse.ArgumentParser(prog="navbot_agent")
    p.add_argument("--host", default=config.WS_HOST)
    p.add_argument("--port", type=int, default=config.WS_PORT)
    p.add_argument("--no-ros", action="store_true",
                   help="run without rclpy/launch control (protocol dev/test)")
    p.add_argument("--motors-off", action="store_true",
                   help="force motors:=false in every mode (safe dev/demo)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    config.FORCE_MOTORS_OFF = args.motors_off
    asyncio.run(_amain(args))


if __name__ == "__main__":
    main()
