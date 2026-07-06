"""WS protocol v1 — JSON text messages + binary video frames.

Full spec: docs/operator_app.md. Binary video frame layout (big-endian):
    [0x01][cam u8][seq u32][agent_mono_ms u32][JPEG bytes]
"""

import json
import struct

PROTO_VERSION = 1
VIDEO_MAGIC = 0x01
VIDEO_HEADER = struct.Struct(">BBII")

CLIENT_TYPES = {"hello", "teleop", "estop", "set_mode", "video", "ping"}


def pack_video(cam_id: int, seq: int, mono_ms: int, jpeg: bytes) -> bytes:
    return VIDEO_HEADER.pack(VIDEO_MAGIC, cam_id,
                             seq & 0xFFFFFFFF, mono_ms & 0xFFFFFFFF) + jpeg


def parse_client(text):
    """Parse one client text message. Returns (msg_dict, None) or (None, error)."""
    try:
        msg = json.loads(text)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None, "invalid JSON"
    if not isinstance(msg, dict) or msg.get("type") not in CLIENT_TYPES:
        return None, f"unknown message type: {msg.get('type') if isinstance(msg, dict) else '?'}"
    return msg, None


def welcome(agent_version, limits, cams, state):
    return {"v": PROTO_VERSION, "type": "welcome", "proto": PROTO_VERSION,
            "agent": f"navbot-agent/{agent_version}",
            "limits": limits, "cams": cams, "state": state}


def state_msg(state):
    return {"v": PROTO_VERSION, "type": "state", **state}


def telemetry(rates, range_cm, health, teleop_age_ms):
    return {"v": PROTO_VERSION, "type": "telemetry", "rates": rates,
            "range_cm": range_cm, "teleop_age_ms": teleop_age_ms, **health}


def sectors(angle_min, angle_max, status, free):
    return {"v": PROTO_VERSION, "type": "sectors", "angle_min": angle_min,
            "angle_max": angle_max, "status": status, "free": free}


def log_msg(src, level, line):
    return {"v": PROTO_VERSION, "type": "log", "src": src,
            "level": level, "line": line}


def pong(t, agent_t):
    return {"v": PROTO_VERSION, "type": "pong", "t": t, "agent_t": agent_t}


def error(msg):
    return {"v": PROTO_VERSION, "type": "error", "msg": msg}
