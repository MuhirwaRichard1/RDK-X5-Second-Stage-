# NavBot Console — desktop operator app

PySide6 app for driving and monitoring the robot from a Windows or Linux PC on
the same network. Talks to the robot-side agent
([app/agent](../agent/README.md)) over WebSocket, port 8080.

![panels: video+HUD left, modes/E-stop/joystick/health right, logs bottom]

## Install & run

Needs Python 3.10+ on the PC.

**Windows (PowerShell):**
```powershell
git clone <this repo> ; cd rdk-x5-navbot\app\desktop
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m navbot_console
```

**Linux:**
```bash
cd rdk-x5-navbot/app/desktop
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 -m navbot_console
```

Enter the robot address (`ws://<robot-ip>:8080`, remembered between runs) and
press **Connect**. On the robot, the agent must be running
(`sudo systemctl start navbot-agent` or `sudo ./app/agent/run_agent.sh`).

## Controls

| control | action |
|---|---|
| mode buttons | STOPPED / OBSERVE (perception, no motors) / MANUAL (drive) / AUTO (avoidance) |
| **Space** | engage E-stop (anywhere; release only by clicking the button) |
| W/S or ↑/↓ | forward / reverse (manual mode) |
| A/D or ←/→ | turn left / right |
| joystick | drag to drive; overrides keys; springs to stop |
| speed slider | scales max speed (10–100 % of 0.40 m/s) |
| camera checkboxes | subscribe front / left / right video |

The sector fan over the front video is the live `/obstacles` map: green = free,
red = blocked, gray = unknown (treated as blocked by the planner). MOTORS
ON/OFF lamp reflects the actual mode; "forward range" turns red under the
30 cm hard-stop threshold.

Driving is only possible while: connected, mode = MANUAL (active), and the
E-stop is released. Losing the connection, alt-tabbing away, or going stale
stops the robot within ~0.5 s (agent staleness + motor dead-man).

## Troubleshooting

- **Connect fails** — agent running? `ping <robot-ip>`; port 8080 free on the
  robot; PC and robot on the same network.
- **Video black but connected** — start a mode (OBSERVE at minimum); the
  cameras only run while a mode is active.
- **Laggy video on weak WiFi** — the console uses `sd` quality by default;
  frames are dropped (never queued) so controls stay responsive.
- **"protocol mismatch"** — update whichever side is older; both must speak v1.
