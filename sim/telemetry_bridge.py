"""PLAN 2.5: stream the Pico's TELEMETRY into PlotJuggler (UDP server, JSON) and keep the heartbeat alive.
In PlotJuggler: Streaming → UDP Server, port 9870, message protocol JSON.
  uv run python telemetry_bridge.py /dev/cu.usbmodemXXXX"""
import json
import socket
import struct
import sys
import time

import link

L = link.Link(sys.argv[1])
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
keys = ("t_us", "x", "x_dot", "theta", "theta_dot", "duty", "mode", "faults", "loop_exec_us", "loop_jitter_us")
last_hb = 0.0
while True:
    now = time.monotonic()
    if now - last_hb > 0.02:                      # 50 Hz heartbeat: well inside the Pico's 100 ms watchdog
        L.heartbeat()
        last_hb = now
    for ptype, _, pl in L.poll():
        if ptype == link.TELEMETRY and len(pl) == struct.calcsize(link.TELEMETRY_FMT):
            d = dict(zip(keys, struct.unpack(link.TELEMETRY_FMT, pl)))
            d["timestamp"] = d["t_us"] / 1e6
            udp.sendto(json.dumps(d).encode(), ("127.0.0.1", 9870))
        elif ptype == link.TEXT:
            print("pico:", pl.decode(errors="replace"))
    time.sleep(0.001)
