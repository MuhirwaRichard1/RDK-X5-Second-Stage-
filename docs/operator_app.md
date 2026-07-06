# Operator app — desktop console + robot agent

The robot is operated from a **desktop app** (`app/desktop`, PySide6, Windows/Linux)
that talks over WiFi to a **robot-side agent** (`app/agent`, Python asyncio +
rclpy) on port **8080**. The agent is the only long-running operator process on
the robot: it serves the WebSocket API, relays teleop/E-stop into the ROS graph,
forwards camera frames, and starts/stops the `ros2 launch` stack behind the
drive modes.

```
 PC (Windows/Linux)                     RDK X5 robot
┌──────────────────┐   WebSocket   ┌─────────────────────────────────────────┐
│  navbot_console  │◄─────:8080───►│  navbot_agent (root)                    │
│  PySide6         │  JSON + JPEG  │   ├─ rclpy node: teleop→/cmd_vel,       │
│  video + HUD     │               │   │   /estop client, telemetry subs     │
│  joystick/WASD   │               │   ├─ launch manager: ros2 launch per    │
│  E-stop, modes   │               │   │   mode (own process group)          │
│  health, logs    │               │   └─ video pump: front JPEG sd/hd,      │
└──────────────────┘               │       sides YUYV→JPEG                   │
                                   └─────────────────────────────────────────┘
```

## Modes

| Mode | Launch file | motors | /cmd_vel publisher |
|---|---|---|---|
| `stopped` | none | — | — |
| `observe` | `navigation.launch.py motors:=false` | off | local_planner (ignored) |
| `manual`  | `manual.launch.py motors:=true` | **on** | **agent teleop only** (no local_planner) |
| `auto`    | `navigation.launch.py motors:=true` | **on** | local_planner |

One mode at a time; switching stops the current launch (SIGINT→SIGTERM→SIGKILL
to the process group) and starts the next. `manual.launch.py` is
`navigation.launch.py` without `local_planner`, so teleop never fights the
planner for `/cmd_vel`. Every command still passes `safety_gate` (TF-Luna
< 30 cm forward stop + `/estop`), and `motor_controller`'s 200 ms dead-man
remains the final backstop.

## Safety model (layered)

1. **Operator E-stop** — latched in the agent; re-asserted on `safety_gate`
   after every mode switch (a reconciler keeps `/estop_state` equal to operator
   intent). Space engages; release is an explicit button click.
2. **Teleop staleness** — agent publishes `/cmd_vel` at 20 Hz only while the
   client feeds it (< 0.4 s old); on silence it sends one zero Twist and stops.
   Client-side, releasing keys/joystick sends a burst of zeros; alt-tabbing
   away releases all keys.
3. **safety_gate** — TF-Luna forward clamp (reverse/rotate still allowed),
   fail-safe when the sensor is unreadable.
4. **motor_controller dead-man** — no `/cmd_vel_safe` for 200 ms → coast stop.
5. **Process hygiene** — launches run in their own process group with a
   pidfile; agent crash/restart sweeps orphans; `systemctl stop navbot-agent`
   kills the whole cgroup.

Agent flag `--motors-off` forces `motors:=false` in every mode (safe demos,
unprivileged development).

## WebSocket protocol v1

One connection per client (several clients allowed). Text = JSON, binary =
video. First client message must be `hello`.

### Client → agent

```jsonc
{"v":1,"type":"hello","client":"navbot-console/0.1"}
{"type":"teleop","vx":0.22,"wz":-0.4,"seq":123}     // m/s, rad/s; agent clamps to limits
{"type":"estop","engage":true}                       // operator latch on/off
{"type":"set_mode","mode":"stopped|observe|manual|auto"}
{"type":"video","cam":"front|left|right","enable":true,
 "fps":15,"quality":"sd|hd"}                         // sd=640x360 q70; hd=native 720p JPEG
{"type":"ping","t":123.456}                          // client monotonic; echoed
```

### Agent → client

```jsonc
{"v":1,"type":"welcome","proto":1,"agent":"navbot-agent/0.1.0",
 "limits":{"v_max":0.40,"w_max":1.2},"cams":["front","left","right"],
 "state":{...}}                                      // + last 200 log lines replayed
{"type":"state","mode":"manual","mode_status":"starting|active|stopping|error",
 "motors":true,"estop":{"latched":true,"confirmed":true},"detail":"..."}
{"type":"telemetry", ...}                            // 2 Hz: rates{topic:Hz},
    // range_cm (null = unreadable/stale), teleop_age_ms, cpu, mem,
    // temp_cpu_c, temp_ddr_c, bpu_pct, wifi_dbm
{"type":"sectors","angle_min":-2.268,"angle_max":2.268,
 "status":[0,1,2,...],"free":[0.0,0.83,...]}         // /obstacles relay @10 Hz
{"type":"log","src":"launch|agent","level":"info|warn|error","line":"..."}
{"type":"pong","t":123.456,"agent_t":98.765}
{"type":"error","msg":"..."}
```

### Binary video frame

```
[0x01][cam u8: 0=front 1=left 2=right][seq u32 BE][agent_mono_ms u32 BE][JPEG]
```

Backpressure: per-client latest-wins frame slot per camera — a slow WiFi
client drops frames, never queues them. Measured on-robot: front `sd`
~10 fps × 21 KB (~1.7 Mbps), front `hd` ~15 fps × 63 KB (~7.4 Mbps), sides
5 fps × 8 KB. Latency is reported as WS round-trip only; robot and PC clocks
are never compared.

## Running

Robot (agent):
```bash
sudo ./app/agent/run_agent.sh                 # foreground, full control
./app/agent/run_agent.sh --motors-off         # unprivileged, motors locked out
sudo ./app/agent/install_service.sh           # install systemd unit (disabled)
sudo systemctl start navbot-agent             # run as a service
sudo systemctl enable navbot-agent            # opt in: start at boot
```

PC (console): see [app/desktop/README.md](../app/desktop/README.md).

Headless API test (also works on the robot):
```bash
python3 app/agent/ws_probe.py ws://<robot-ip>:8080 --mode observe --watch 30
python3 app/agent/ws_probe.py ws://<robot-ip>:8080 --video front --dump-frames /tmp/f
```

## First motors-on checklist (do once, in order)

Everything above was verified motors-off on the robot. Before the first real
drive from the console:

1. **Wheels off the ground.** Agent via `sudo ./app/agent/run_agent.sh`
   (no `--motors-off`). Console → MANUAL. Verify: MOTORS ON lamp lights only
   when the mode is active.
2. Push the joystick forward → wheels spin; release → stop < 0.5 s.
3. **Space** mid-drive → immediate stop; wheels stay dead until RELEASE.
4. Kill the console (or PC WiFi) mid-drive → wheels stop ≤ ~0.6 s
   (0.4 s agent staleness + 0.2 s motor dead-man).
5. Hand in front of the TF-Luna → forward refuses, reverse still works.
6. On the floor at low speed slider (≤ 30 %), repeat 2–3.
7. AUTO mode in a clear area, finger on Space.

## Files

- `app/agent/navbot_agent/` — `server.py` (WS sessions/backpressure),
  `ros_bridge.py` (rclpy node, teleop timer, E-stop reconciler),
  `launch_manager.py` (mode state machine, orphan sweeping),
  `video.py`, `health.py`, `protocol.py`, `config.py`
- `app/desktop/navbot_console/` — `client.py` (QWebSocket + reconnect),
  `teleop.py` (20 Hz key/joystick merge), `main_window.py`, `widgets/`
- ROS additions: `safety_gate` now publishes `/estop_state`
  (latched Bool) + `/range_forward` (Range, NaN = unreadable);
  new `navbot_bringup/launch/manual.launch.py`
