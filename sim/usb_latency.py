"""PLAN 2.5: USB round-trip latency to the Pico. 10,000 PINGs; median, p99, max → analysis/usb-latency.md.
  uv run python usb_latency.py /dev/cu.usbmodemXXXX"""
import struct
import sys
import time

import numpy as np

import link
import model

L = link.Link(sys.argv[1])
rtts = []
for i in range(10_000):
    t0 = time.perf_counter_ns()
    L.send(link.PING, struct.pack("<IQ", i, t0))
    got = None
    while got is None and time.perf_counter_ns() - t0 < 50_000_000:
        for t, _, pl in L.poll():
            if t == link.PONG and struct.unpack("<I", pl[:4])[0] == i:
                got = time.perf_counter_ns()
    if got:
        rtts.append((got - t0) / 1000)
    if i % 50 == 0:
        L.heartbeat()
r = np.array(rtts)
text = (f"# USB round-trip latency (PLAN 2.5)\n\n{len(r)}/10000 replies. Round trip (µs): median {np.median(r):.0f}, "
        f"p99 {np.percentile(r, 99):.0f}, max {r.max():.0f}.\n")
out = model.SIM_DIR.parent / "analysis" / "usb-latency.md"
out.parent.mkdir(exist_ok=True)
out.write_text(text)
print(text)
