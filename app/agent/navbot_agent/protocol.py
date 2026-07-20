"""WS protocol v1 — JSON text messages + binary video frames — plus the
UDP fast path (v1.1) used for latency-critical traffic, GCS-style.

Full spec: docs/operator_app.md. Binary video frame layout (big-endian):
    [0x01][cam u8][seq u32][agent_mono_ms u32][JPEG bytes]

UDP datagrams (big-endian). Uplink starts [magic u8][token 8B] where the
token comes from the WS welcome message and binds the datagram to a session:
    0x10 PING    + [t_client f64]
    0x11 TELEOP  + [seq u32][vx f32][wz f32]
    0x12 ESTOP   + [seq u32][engage u8]
Downlink:
    0x20 PONG    [t_client f64][agent_mono f64]
    0x21 JSON    [utf-8 JSON]                     (telemetry / sectors)
    0x22 VIDEO   [cam u8][seq u16][mono_ms u32][frag u8][nfrags u8][chunk]
"""

import json
import struct

PROTO_VERSION = 1
VIDEO_MAGIC = 0x01
VIDEO_HEADER = struct.Struct(">BBII")

UDP_PING = 0x10
UDP_TELEOP = 0x11
UDP_ESTOP = 0x12
UDP_PONG = 0x20
UDP_JSON = 0x21
UDP_VIDEO = 0x22

UDP_PING_S = struct.Struct(">B8sd")
UDP_TELEOP_S = struct.Struct(">B8sIff")
UDP_ESTOP_S = struct.Struct(">B8sIB")
UDP_PONG_S = struct.Struct(">Bdd")
UDP_VIDEO_S = struct.Struct(">BBHIBB")

UDP_CHUNK = 1200            # video fragment payload — fits any sane MTU

CLIENT_TYPES = {"hello", "teleop", "estop", "set_mode", "video", "ping",
                "set_model", "set_map", "save_map", "set_goal"}


def pack_video(cam_id: int, seq: int, mono_ms: int, jpeg: bytes) -> bytes:
    return VIDEO_HEADER.pack(VIDEO_MAGIC, cam_id,
                             seq & 0xFFFFFFFF, mono_ms & 0xFFFFFFFF) + jpeg


def pack_udp_json(msg: dict) -> bytes:
    return bytes((UDP_JSON,)) + json.dumps(msg).encode()


def fragment_video(cam_id: int, seq: int, mono_ms: int, jpeg: bytes):
    """Split one JPEG into UDP datagrams; receiver drops incomplete frames."""
    nfrags = max(1, -(-len(jpeg) // UDP_CHUNK))
    if nfrags > 255:                       # > ~300 KB frame — never on this bot
        return []
    return [UDP_VIDEO_S.pack(UDP_VIDEO, cam_id, seq & 0xFFFF,
                             mono_ms & 0xFFFFFFFF, i, nfrags)
            + jpeg[i * UDP_CHUNK:(i + 1) * UDP_CHUNK]
            for i in range(nfrags)]


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


def telemetry(rates, range_cm, health, teleop_age_ms, odom=None):
    """`odom` = {"source": "icp"|"dr"|"fused"|None, "pose_age_ms": int|None} —
    the live SLAM odom backbone and how fresh the map->base_link fix is, so the
    console can show the source and flag lost localization (see map_panel)."""
    return {"v": PROTO_VERSION, "type": "telemetry", "rates": rates,
            "range_cm": range_cm, "teleop_age_ms": teleop_age_ms,
            "odom": odom or {}, **health}


def sectors(angle_min, angle_max, status, free):
    return {"v": PROTO_VERSION, "type": "sectors", "angle_min": angle_min,
            "angle_max": angle_max, "status": status, "free": free}


def grid_overlay(camera, kind, rows, cols, cells):
    """Coarse per-cell HUD overlay — pidnet segmentation or depth, same
    shape of data, distinguished by `kind` (0=pidnet, 1=depth)."""
    return {"v": PROTO_VERSION, "type": "grid_overlay", "camera": camera,
            "kind": kind, "rows": rows, "cols": cols, "cells": cells}


def detections(camera, boxes):
    """YOLO11 boxes for one camera frame. `boxes` is a list of
    {x1,y1,x2,y2,score,class_name} dicts, coords normalized 0..1."""
    return {"v": PROTO_VERSION, "type": "detections", "camera": camera,
            "boxes": boxes}


def map_msg(seq, width, height, png_b64, resolution=0.0,
            origin_x=0.0, origin_y=0.0):
    """SLAM occupancy-grid snapshot, rendered agent-side to a small grayscale
    PNG (robot marker already baked in when a fresh map->base_link TF is
    available) — the desktop just decodes and displays. resolution + origin let
    the console turn a pixel click into a map-frame (x, y) goal. The PNG is
    flipped so image-up = world +y (see map_pump._render)."""
    return {"v": PROTO_VERSION, "type": "map", "seq": seq,
            "width": width, "height": height, "png_b64": png_b64,
            "resolution": resolution, "origin_x": origin_x, "origin_y": origin_y}


def attitude(roll_deg, pitch_deg, yaw_deg, yaw_rate_dps):
    """10 Hz orientation for the console instruments. Yaw is gyro-integrated
    (no magnetometer) — relative to power-on heading, drifts slowly."""
    return {"v": PROTO_VERSION, "type": "att", "roll": roll_deg,
            "pitch": pitch_deg, "yaw": yaw_deg, "yaw_rate": yaw_rate_dps}


def log_msg(src, level, line):
    return {"v": PROTO_VERSION, "type": "log", "src": src,
            "level": level, "line": line}


def pong(t, agent_t):
    return {"v": PROTO_VERSION, "type": "pong", "t": t, "agent_t": agent_t}


def error(msg):
    return {"v": PROTO_VERSION, "type": "error", "msg": msg}
