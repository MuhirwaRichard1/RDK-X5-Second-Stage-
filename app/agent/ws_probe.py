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
"""

import argparse
import asyncio
import json
import os
import statistics
import struct
import time

import websockets

HDR = struct.Struct(">BBII")
CAMS = {0: "front", 1: "left", 2: "right"}


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
    args = ap.parse_args()

    frames = {}                                   # cam -> [count, bytes]
    async with websockets.connect(args.url, max_size=4 * 1024 * 1024) as ws:
        await ws.send(json.dumps({"v": 1, "type": "hello",
                                  "client": "ws-probe/0.1"}))
        welcome = json.loads(await ws.recv())
        print(f"WELCOME {welcome['agent']} proto={welcome['proto']} "
              f"limits={welcome['limits']} state={welcome['state']}")

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
                await ws.send(json.dumps({
                    "type": "teleop", "vx": 0.3 * math.sin(ph),
                    "wz": 0.8 * math.cos(ph), "seq": seq}))
                seq += 1
                await asyncio.sleep(0.05)
            print("TELEOP sine done (now silent — expect zero+stop)")

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

    for name, (n, size) in frames.items():
        print(f"FRAMES {name}: {n} in {args.watch:.0f}s "
              f"(~{n / args.watch:.1f} fps, avg {size // max(n, 1) // 1024} KB)")


if __name__ == "__main__":
    asyncio.run(main())
