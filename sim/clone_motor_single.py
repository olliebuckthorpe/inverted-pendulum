"""Motor choice check (29 Sep): Pololu #4757 vs a JGB37-520 clone, modelled from seller figures (2.3 A stall at
12 V), in the datasheet sim: single swing-up success/time over 20 starts, and the kick ladder.
Note the kick is "duty overridden to +1" for D ms, so a weaker motor gets a weaker kick: the ladder numbers are
NOT comparable between motors.   Run from sim/:  uv run python clone_motor_single.py"""
import math, sys
import numpy as np
sys.path.insert(0, ".")
import lqr, model, swingup
from plant import Plant, wrap

def clone(rpm, stall_a=2.3, i0=0.15):
    w = rpm * 2 * math.pi / 60
    R = 12 / stall_a
    Ke = (12 - i0 * R) / w
    return dict(motor__R=R, motor__Kt=Ke, motor__Ke=Ke, motor__I_noload=i0)

CASES = {"Pololu 4757 (1600 rpm)": {},
         "JGB37-520 1600 rpm": clone(1600),
         "JGB37-520 1000 rpm": clone(1000)}

def run(P, K, seed, secs, start, kick=None):
    c = swingup.EnergySwingUp(P, K)
    if abs(start[1]) < 1: c.mode = 1
    last = [0.0]
    def ctrl(t, o):
        u = float(np.clip(c(t, o), -1, 1))
        if kick and kick[0] <= t < kick[0] + kick[1]: u = 1.0
        u = float(np.clip(u, last[0] - .25, last[0] + .25)); last[0] = u; return u
    ctrl.mode = 0
    pl = Plant(P, rate_hz=500, seed=seed); pl.reset(x=start[0], th=start[1])
    L = pl.run(ctrl, secs, stop=lambda t, d: abs(d.qpos[0]) > 0.395)
    L["th_w"] = np.array([wrap(a) for a in L["th"]]); return L

for name, ov in CASES.items():
    P = model.with_params(model.load_params(), supply__current_cap=4.5, sensing__angle_noise_deg=0.068, **ov)
    K = lqr.design(P, 500)
    rng = np.random.default_rng(0); ok, times = 0, []
    for s in range(20):
        L = run(P, K, s, 15.0, (rng.uniform(-.02, .02), math.pi + rng.uniform(-.03, .03)))
        bad = np.where(np.abs(L["th_w"]) >= math.radians(10))[0]
        if L["t"][-1] >= 14.99 and len(bad) and bad[-1] + 1 < len(L["t"]) and L["t"][-1] - L["t"][bad[-1]] > 5:
            ok += 1; times.append(L["t"][bad[-1] + 1])
    best = 0
    for D in range(20, 401, 20):
        n = sum(bool((L := run(P, K, 1000 + s, 6.0, (rng.uniform(-.01, .01), rng.uniform(-.01, .01)), kick=(1.0, D / 1000)))["t"][-1] >= 5.99
                     and np.max(np.abs(L["th_w"])) < math.radians(35) and abs(L["th_w"][-1]) < math.radians(10)) for s in range(10))
        if n >= 8: best = D
        if n == 0: break
    print(f"{name:26s} stall torque {model.with_params(model.load_params(), **ov)['motor']['Kt']*12/model.with_params(model.load_params(), **ov)['motor']['R']:.3f} N·m | "
          f"swing-ups {ok}/20, median {np.median(times) if times else float('nan'):.1f} s | largest kick survived {best} ms", flush=True)
