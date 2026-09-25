"""Plan a transition between two resting positions of the double (DD, UU, UD, DU), robustly:
duty <= 0.6, cart within ±300 mm, joint-2 friction assumed x1.5, rod 1 within ±1 turn of hanging (cable).
Usage: uv run python plan_transition.py UU UD     -> results/trajectories/T_UU-UD.npz"""
import math
import sys

import double_control as C
import double_trajopt as D
import model

src, dst = sys.argv[1], sys.argv[2]
start, end = C.EQUILIBRIA[src], C.EQUILIBRIA[dst]
p = D.layout_params(30, 0.30, 0.20)
p = model.with_params(p, joint2__damping=p["joint2"]["damping"] * 1.5)


def nearest(target: float, ref: float) -> float:
    """The copy of `target` (mod 2π) closest to `ref`: turn the short way round."""
    return target + 2 * math.pi * round((ref - target) / (2 * math.pi))


a1e, a2e = nearest(end[0], start[0]), nearest(end[1], start[1])
tries = [(a1e, a2e), (a1e + 2 * math.pi, a2e), (a1e - 2 * math.pi, a2e), (a1e, a2e + 2 * math.pi), (a1e, a2e - 2 * math.pi)]
for a1, a2 in tries:
    if not (-math.pi + 0.2 <= a1 <= 3 * math.pi - 0.2):
        continue
    r = D.continuation(p, D.Limits(u_max=0.6, x_max=0.30), steps=6, a1_end=a1, a2_end=a2, start=start)
    if r["ok"]:
        D.save(r, f"T_{src}-{dst}")
        print(f"{src}->{dst}: FOUND {r['T']:.2f} s (end a1={math.degrees(a1):.0f}, a2={math.degrees(a2):.0f} deg), "
              f"max|x| {r['max_x'] * 1000:.0f} mm", flush=True)
        break
    print(f"{src}->{dst}: end ({math.degrees(a1):.0f},{math.degrees(a2):.0f}) not found ({r['reached'] * 100:.0f}%)", flush=True)
else:
    print(f"{src}->{dst}: NOT FOUND", flush=True)
