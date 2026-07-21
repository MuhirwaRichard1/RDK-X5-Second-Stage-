# Operator app — desktop console + robot agent

The robot is operated from a **desktop app** (`app/desktop`, PySide6, Windows/Linux)
that talks over WiFi to a **robot-side agent** (`app/agent`, Python asyncio +
rclpy) on port **8080**. The agent is the only long-running operator process on
the robot: it serves the WebSocket API, relays teleop/E-stop into the ROS graph,
forwards camera frames, and starts/stops the `ros2 launch` stack behind the
drive modes.

Transport is split GCS-style (like MAVLink ground stations): the WebSocket
(TCP :8080) is the reliable control plane — hello/welcome, mode switching,
state, logs — while all latency-critical traffic rides a **UDP fast path on
:8080/udp**: teleop up; telemetry, sectors and video down. UDP never
retransmits, so a lost packet costs one 50 ms teleop tick or one video frame
instead of a TCP head-of-line stall, and video can't build up seconds of
latency in the kernel's TCP send buffer. If UDP is blocked (firewall/NAT),
everything falls back to the WebSocket automatically within 3 s.

```
 PC (Windows/Linux)                     RDK X5 robot
┌──────────────────┐  WS :8080/tcp ┌─────────────────────────────────────────┐
│  navbot_console  │◄─────────────►│  navbot_agent (root)                    │
│  PySide6         │ hello, modes, │   ├─ rclpy node: teleop→/cmd_vel,       │
│  video + HUD     │ state, logs   │   │   /estop client, telemetry subs     │
│  joystick/WASD   │  UDP :8080    │   ├─ launch manager: ros2 launch per    │
│  E-stop, modes   │◄─────────────►│   │   mode (own process group)          │
│  health, logs    │ teleop↑ telem │   └─ video pump: front JPEG sd/hd,      │
└──────────────────┘ +video frags↓ │       sides YUYV→JPEG                   │
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
{"type":"set_mode","mode":"stopped|observe|manual|auto|mapping|navigate",
 "map":"kitchen"}                                    // map: navigate only — saved map to localize against
{"type":"set_map","enable":true}                     // stream /map PNGs to this session
{"type":"save_map","name":"kitchen"}                 // save live map as <name> (blank -> "current")
{"type":"delete_map","name":"kitchen"}               // delete a saved map (all <name>.* files)
{"type":"set_goal","x":1.2,"y":-0.4}                 // navigate: drive to map-frame point (console map click)
{"type":"set_model","model":"obstacle_avoidance|yolo11|depthanything","enable":true}
{"type":"video","cam":"front|left|right","enable":true,
 "fps":15,"quality":"sd|hd"}                         // sd=640x360 q70; hd=native 720p JPEG
{"type":"ping","t":123.456}                          // client monotonic; echoed
```

### Agent → client

```jsonc
{"v":1,"type":"welcome","proto":1,"agent":"navbot-agent/0.1.0",
 "limits":{"v_max":0.40,"w_max":1.2},"cams":["front","left","right"],
 "udp":{"port":8080,"token":"9f3a…"},                // UDP fast path offer (v1.1)
 "state":{...}}                                      // + last 200 log lines replayed
{"type":"state","mode":"manual","mode_status":"starting|active|stopping|error",
 "motors":true,"estop":{"latched":true,"confirmed":true},"detail":"...",
 "models":{...},"maps":["arena","kitchen"]}          // maps: saved maps for the NAVIGATE picker
{"type":"telemetry", ...}                            // 2 Hz: rates{topic:Hz},
    // range_cm (null = unreadable/stale), teleop_age_ms, cpu, mem,
    // temp_cpu_c, temp_ddr_c, bpu_pct, wifi_dbm,
    // odom:{source:"icp"|"dr"|"fused"|null, pose_age_ms:int|null}
    //   source = live SLAM odom backbone (null = no SLAM mode running);
    //   pose_age_ms = age of last map->base_link fix (null = never localized,
    //   > 1500 = tracking/localization lost). Console shows it in the health
    //   panel + a POSE-LOST badge on the map (relocalizing in navigate/kidnap).
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
5 fps × 8 KB. Latency is reported as link round-trip only; robot and PC clocks
are never compared.

### UDP fast path (v1.1)

The welcome message offers `udp:{port, token}`; the token is 8 random bytes
per WS session (hex-encoded). Any datagram carrying it binds the sender's
address to that session — the client just starts pinging and everything
reroutes automatically; the binding follows the client if its address changes.
All integers big-endian; uplink packets start `[magic u8][token 8B]`:

```
uplink    0x10 PING    [t_client f64]
          0x11 TELEOP  [seq u32][vx f32][wz f32]      // reordered pkts dropped
          0x12 ESTOP   [seq u32][engage u8]           // sent x3 + WS copy
downlink  0x20 PONG    [t_client f64][agent_mono f64]
          0x21 JSON    [utf-8 JSON]                   // telemetry / sectors
          0x22 VIDEO   [cam u8][seq u16][mono_ms u32][frag u8][nfrags u8][chunk]
```

Video frames are split into ≤ 1200-byte fragments; the receiver reassembles
and drops any frame still incomplete when a newer one starts — one lost
packet = one dropped frame, never a stall. Rules: agent uses UDP downlink
for a session while a client datagram arrived < 3 s ago; the console uses
UDP uplink for teleop while a UDP pong/datagram arrived < 2.5 s ago; state,
logs and mode commands always stay on the WS. E-stop goes on **both**
channels (idempotent, agent dedupes by seq). The HUD's "link RTT" row shows
which transport is live (`· UDP` / `· TCP`). The token binds datagrams to a
session; it is not authentication — trust model is unchanged (LAN).

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
python3 app/agent/ws_probe.py ws://<robot-ip>:8080 --udp --ping 10 --video front  # fast path
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

- `app/agent/navbot_agent/` — `server.py` (WS sessions/backpressure + UDP
  fast path), `ros_bridge.py` (rclpy node, teleop timer, E-stop reconciler),
  `launch_manager.py` (mode state machine, orphan sweeping),
  `video.py`, `health.py`, `protocol.py`, `config.py`
- `app/desktop/navbot_console/` — `client.py` (QWebSocket + reconnect),
  `teleop.py` (20 Hz key/joystick merge), `main_window.py`, `widgets/`
- ROS additions: `safety_gate` now publishes `/estop_state`
  (latched Bool) + `/range_forward` (Range, NaN = unreadable);
  new `navbot_bringup/launch/manual.launch.py`
