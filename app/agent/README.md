# navbot_agent — robot-side operator agent

Serves the WebSocket API (port 8080) the desktop console connects to, plus a
UDP fast path on 8080/udp for teleop/telemetry/video (GCS-style; automatic
WS fallback), and owns the `ros2 launch` lifecycle for the drive modes.
Protocol + architecture: [docs/operator_app.md](../../docs/operator_app.md).

## Run

```bash
sudo ./run_agent.sh                # full agent (modes may enable motors)
./run_agent.sh --motors-off        # unprivileged; motors forced off everywhere
python3 -m navbot_agent --no-ros   # protocol/server only (no rclpy), from app/agent
```

Root is only needed for the motor path (PWM sysfs writes by motor_controller);
`--motors-off` runs fine as `sunrise`.

## systemd (optional)

```bash
sudo ./install_service.sh          # installs unit, leaves it DISABLED
sudo systemctl start navbot-agent
sudo systemctl enable navbot-agent # opt in to boot start
journalctl -u navbot-agent -f
```

Stopping the service kills the agent **and** any launch it started (cgroup),
so nothing keeps driving after a stop.

## Test without the desktop app

```bash
python3 ws_probe.py ws://127.0.0.1:8080 --watch 10                 # telemetry
python3 ws_probe.py ws://127.0.0.1:8080 --mode observe --watch 40  # start stack
python3 ws_probe.py ws://127.0.0.1:8080 --estop true --watch 2
python3 ws_probe.py ws://127.0.0.1:8080 --video front --quality sd \
        --dump-frames /tmp/frames --watch 5
python3 ws_probe.py ws://127.0.0.1:8080 --teleop-sine 5 --watch 8  # manual mode only
python3 ws_probe.py ws://127.0.0.1:8080 --udp --ping 10 --watch 8  # UDP fast path:
        # prints UDP vs WS RTT; teleop/telemetry/video counters ride UDP
```

## Layout

| file | role |
|---|---|
| `navbot_agent/config.py` | ports, limits (v_max/w_max), modes, camera/topic tables |
| `navbot_agent/protocol.py` | message schema v1 + binary video header + UDP packet formats |
| `navbot_agent/server.py` | websockets server + UDP fast path; per-client outbox + latest-wins video slots |
| `navbot_agent/app.py` | operator state (mode, E-stop latch, log ring), telemetry loop |
| `navbot_agent/ros_bridge.py` | rclpy node in a thread: subs, 20 Hz teleop, E-stop reconciler |
| `navbot_agent/launch_manager.py` | mode transitions, process-group kill, orphan sweep, watchdog |
| `navbot_agent/video.py` | front JPEG passthrough/sd re-encode, sides YUYV→JPEG |
| `navbot_agent/health.py` | CPU/mem/temps/BPU/WiFi sampling |
| `ws_probe.py` | headless test client (the agent's verification driver) |
