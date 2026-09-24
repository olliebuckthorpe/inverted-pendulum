"""Plan the 30T 30/20 double swing-up with ONE parameter scaled, e.g. joint-2 friction ×1.3, and save it.
Usage: uv run python plan_variant.py joint2.damping 1.3 [u_max] [x_max]"""
import sys

import double_trajopt as D
import model

key, factor = sys.argv[1], float(sys.argv[2])
sec, k = key.split(".")
p = D.layout_params(30, 0.30, 0.20)
p = model.with_params(p, **{f"{sec}__{k}": p[sec][k] * factor})
u_max = float(sys.argv[3]) if len(sys.argv) > 3 else D.Limits().u_max
x_max = float(sys.argv[4]) if len(sys.argv) > 4 else D.Limits().x_max
r = D.continuation(p, D.Limits(u_max=u_max, x_max=x_max), steps=6)
name = f"30T_30-20_{k}{int(round(factor * 100))}"
if len(sys.argv) > 3:
    name += f"_u{int(round(u_max * 100))}_x{int(round(x_max * 100))}"
if r["ok"]:
    D.save(r, name)
print(f"{name}: {'FOUND ' + format(r['T'], '.2f') + ' s' if r['ok'] else 'not found'}", flush=True)
