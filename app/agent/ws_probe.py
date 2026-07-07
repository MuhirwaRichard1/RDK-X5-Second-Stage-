#!/usr/bin/env python3
"""ws_probe — headless test client for the navbot agent (verification driver).

    python3 ws_probe.py ws://127.0.0.1:8080 [options]

Options (combinable; probe exits when --watch expires):
    --watch S           print state/telemetry/log/sectors for S seconds (default 5)
    --ping N            send N pings first, print RTT stats
    --estop true|false  send an estop command
    --mode M            send set_mode (stopped|observe|manual|auto)
    --video CAM         subscribe to a camera (front|left|right)
    --quality sd|hd     video quality for --video
    --dump-frames DIR   write received JPEG frames to DIR
    --teleop-sine S     send a vx/wz sine wave for S seconds at 20 Hz
    --slow MS           sleep MS per received message (simulates slow client)
    --udp               use the UDP fast path (ping/teleop up, telemetry/video
                        down) like the console does; prints UDP RTT + counters
"""

import argparse
import asyncio
import json
import os
import statistics
import struct
import time
from urllib.parse import urlparse

import websockets

HDR = struct.Struct(">BBII")
CAMS = {0: "front", 1: "left", 2: "right"}

U_PING = struct.Struct(">B8sd")
U_TELEOP = struct.Struct(">B8sIff")
U_PONG = struct.Struct(">Bdd")
U_VIDEO = struct.Struct(">BBHIBB")


class UdpProbe(asyncio.DatagramProtocol):
    """Client side of the UDP fast path: sends pings, reassembles video."""

    def __init__(self, token, dump_dir=None):
        self.token = token
        self.dump_dir = dump_dir
        self.transport = None
        self.rtts = []
        self.n_json = 0
        self.json_types = {}
        self.frames = {}                     # cam -> [count, bytes]
        self.frags = {}                      # cam -> [seq, nfrags, {idx: chunk}]
        self.incomplete = 0

    def connection_made(self, transport):
        self.transport = transport

    def ping(self):
        if self.transport:
            self.transport.sendto(U_PING.pack(0x10, self.token, time.monotonic()))

    def teleop(self, seq, vx, wz):
        if self.transport:
            self.transport.sendto(U_TELEOP.pack(0x11, self.token, seq, vx, wz))

    def datagram_received(self, data, addr):
        magic = data[0]
        if magic == 0x20 and len(data) == U_PONG.size:
            _, t, _ = U_PONG.unpack(data)
            self.rtts.append((time.monotonic() - t) * 1000)
        elif magic == 0x21:
            self.n_json += 1
            t = json.loads(data[1:]).get("type")
            self.json_types[t] = self.json_types.get(t, 0) + 1
        elif magic == 0x22 and len(data) > U_VIDEO.size:
            _, cam, seq, mono, idx, nfrags = U_VIDEO.unpack(data[:U_VIDEO.size])
            st = self.frags.get(cam)
            if st is None or ((seq - st[0]) & 0xFFFF) < 0x8000 and seq != st[0]:
                if st is not None and len(st[2]) < st[1]:
                    self.incomplete += 1
                st = self.frags[cam] = [seq, nfrags, {}]
            elif seq != st[0]:
                return
            st[2][idx] = data[U_VIDEO.size:]
            if len(st[2]) == nfrags:
                jpeg = b"".join(st[2][i] for i in range(nfrags))
                del self.frags[cam]
                name = CAMS.get(cam, "?")
                c = self.frames.setdefault(name, [0, 0])
                c[0] += 1
                c[1] += len(jpeg)
                if self.dump_dir:
                    with open(os.path.join(self.dump_dir,
                                           f"udp_{name}_{seq:06d}.jpg"), "wb") as f:
                        f.write(jpeg)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("--watch", type=float, default=5.0)
    ap.add_argument("--ping", type=int, default=0)
    ap.add_argument("--estop", choices=["true", "false"])
    ap.add_argument("--mode")
    ap.add_argument("--video", choices=["front", "left", "right"])
    ap.add_argument("--quality", choices=["sd", "hd"], default="sd")
    ap.add_argument("--dump-frames")
    ap.add_argument("--teleop-sine", type=float, default=0.0)
    ap.add_argument("--slow", type=float, default=0.0)
    ap.add_argument("--udp", action="store_true")
    args = ap.parse_args()

    frames = {}                                   # cam -> [count, bytes]
    udp = None
    async with websockets.connect(args.url, max_size=4 * 1024 * 1024) as ws:
        await ws.send(json.dumps({"v": 1, "type": "hello",
                                  "client": "ws-probe/0.1"}))
        welcome = json.loads(await ws.recv())
        print(f"WELCOME {welcome['agent']} proto={welcome['proto']} "
              f"limits={welcome['limits']} state={welcome['state']}")

        udp_task = None
        if args.udp:
            info = welcome.get("udp")
            if not info:
                print("UDP: agent did not advertise a fast path")
            else:
                host = urlparse(args.url).hostname
                _, udp = await asyncio.get_running_loop().create_datagram_endpoint(
                    lambda: UdpProbe(bytes.fromhex(info["token"]),
                                     args.dump_frames),
                    remote_addr=(host, info["port"]))

                async def udp_pinger():
                    while True:
                        udp.ping()
                        await asyncio.sleep(1.0)
                udp_task = asyncio.create_task(udp_pinger())
                print(f"UDP: fast path to {host}:{info['port']}")

        if args.ping:
            rtts = []
            for _ in range(args.ping):
                t0 = time.monotonic()
                await ws.send(json.dumps({"type": "ping", "t": t0}))
                while True:
                    m = await ws.recv()
                    if isinstance(m, str) and json.loads(m).get("type") == "pong":
                        rtts.append((time.monotonic() - t0) * 1000)
                        break
                await asyncio.sleep(0.05)
            print(f"PING n={len(rtts)} mean={statistics.mean(rtts):.2f}ms "
                  f"max={max(rtts):.2f}ms")

        if args.estop:
            await ws.send(json.dumps({"type": "estop",
                                      "engage": args.estop == "true"}))
        if args.mode:
            await ws.send(json.dumps({"type": "set_mode", "mode": args.mode}))
        if args.video:
            await ws.send(json.dumps({"type": "video", "cam": args.video,
                                      "enable": True, "quality": args.quality}))
        if args.dump_frames:
            os.makedirs(args.dump_frames, exist_ok=True)

        async def teleop_task():
            import math
            t0 = time.monotonic()
            seq = 0
            while time.monotonic() - t0 < args.teleop_sine:
                ph = (time.monotonic() - t0) * 2 * math.pi / 4.0
                vx, wz = 0.3 * math.sin(ph), 0.8 * math.cos(ph)
                if udp:
                    udp.teleop(seq, vx, wz)
                else:
                    await ws.send(json.dumps({"type": "teleop", "vx": vx,
                                              "wz": wz, "seq": seq}))
                seq += 1
                await asyncio.sleep(0.05)
            print(f"TELEOP sine done over {'UDP' if udp else 'WS'} "
                  "(now silent — expect zero+stop)")

        teleop = asyncio.create_task(teleop_task()) if args.teleop_sine else None
        t_end = time.monotonic() + args.watch
        n_sectors = 0
        while time.monotonic() < t_end:
            try:
                m = await asyncio.wait_for(ws.recv(), timeout=max(0.1, t_end - time.monotonic()))
            except asyncio.TimeoutError:
                break
            if args.slow:
                await asyncio.sleep(args.slow / 1000.0)
            if isinstance(m, (bytes, bytearray)):
                magic, cam, seq, mono = HDR.unpack(m[:HDR.size])
                name = CAMS.get(cam, "?")
                c = frames.setdefault(name, [0, 0])
                c[0] += 1
                c[1] += len(m)
                if args.dump_frames:
                    with open(os.path.join(args.dump_frames,
                                           f"{name}_{seq:06d}.jpg"), "wb") as f:
                        f.write(m[HDR.size:])
                continue
            msg = json.loads(m)
            t = msg.get("type")
            if t == "sectors":
                n_sectors += 1
                if n_sectors % 20 == 1:
                    st = msg["status"]
                    print(f"SECTORS n={len(st)} free={st.count(1)} "
                          f"blocked={st.count(2)} unknown={st.count(0)}")
            elif t in ("state", "telemetry", "log", "error"):
                print(f"{t.upper()} {json.dumps({k: v for k, v in msg.items() if k not in ('v', 'type')})}")
        if teleop:
            teleop.cancel()
        if udp_task:
            udp_task.cancel()

    for name, (n, size) in frames.items():
        print(f"FRAMES {name} (WS): {n} in {args.watch:.0f}s "
              f"(~{n / args.watch:.1f} fps, avg {size // max(n, 1) // 1024} KB)")
    if udp:
        if udp.rtts:
            print(f"UDP PING n={len(udp.rtts)} mean={statistics.mean(udp.rtts):.2f}ms "
                  f"max={max(udp.rtts):.2f}ms")
        print(f"UDP JSON n={udp.n_json} {udp.json_types}")
        for name, (n, size) in udp.frames.items():
            print(f"FRAMES {name} (UDP): {n} in {args.watch:.0f}s "
                  f"(~{n / args.watch:.1f} fps, avg {size // max(n, 1) // 1024} KB) "
                  f"incomplete={udp.incomplete}")


if __name__ == "__main__":
    asyncio.run(main())
