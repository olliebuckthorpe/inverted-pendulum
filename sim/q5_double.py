"""SPEC §11 Q5 for the double: which layout, and what loop rate — for balance AND swing-up.

For every layout with a saved swing-up trajectory (double_trajopt.py), and the balance-only case:
  - swing-up + hand-over: successes out of 10 randomised starts, supply capped at 5 A, at 1 kHz and 500 Hz
  - balance: the largest tap on rod 1 (sudden spin, rad/s) the balance LQR recovers from, at 1 kHz and 500 Hz

Run:  uv run python q5_double.py
"""

from __future__ import annotations

import json
import math

import numpy as np

import double_control as C
import double_trajopt as D
import model
from plant import wrap

CASES = [f"{t}T_{b}-{u}" for t in (30, 20) for b, u in ((30, 20), (30, 30), (20, 30), (34, 20))]


def params_for(name: str) -> dict:
    teeth, rods = name.split("_")
    p = D.layout_params(int(teeth[:-1]), *(int(v) / 100 for v in rods.split("-")))
    return model.with_params(p, supply__current_cap=5.0)


def swingup_score(p: dict, traj: dict, rate: float, n: int = 10) -> tuple[int, float]:
    g = C.tvlqr(p, traj, rate)
    ok, max_x = 0, 0.0
    for seed in range(n):
        rng = np.random.default_rng(seed)
        start = (rng.uniform(-0.01, 0.01), math.pi + rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
        r = C.trial(p, traj, rate_hz=rate, gains=g, start=start, seed=seed)
        ok += r["ok"]
        max_x = max(max_x, r["max_x"])
    return ok, max_x


def survives_tap(p: dict, K: np.ndarray, kick: float, rate: float) -> bool:
    for sign in (1, -1):
        plant = C.DoublePlant(p, rate_hz=rate)
        plant.reset(q=(0.0, 0.0, 0.0))

        class Bal:
            mode = 1
            tapped = False

            def __call__(self, t, s):
                if not self.tapped and t >= 0.5:
                    plant.d.qvel[1] += sign * kick
                    self.tapped = True
                e = s.copy()
                e[1], e[2] = wrap(e[1]), wrap(e[2])
                return float(-K.ravel() @ e)
        log = plant.run(Bal(), 3.5, stop=lambda d: abs(d.qpos[0]) > 0.395 or abs(wrap(d.qpos[1])) > 1.0)
        if log["t"][-1] < 3.5 - 2 * plant.dt:
            return False
        if abs(wrap(log["q"][-1, 1])) > math.radians(5) or abs(wrap(log["q"][-1, 2])) > math.radians(5):
            return False
    return True


def largest_tap(p: dict, rate: float) -> float:
    K, _ = C.balance_gain(p, rate)
    lo, hi = 0.0, 8.0
    if survives_tap(p, K, hi, rate):
        return hi
    for _ in range(10):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if survives_tap(p, K, mid, rate) else (lo, mid)
    return lo


if __name__ == "__main__":
    out = {}
    for name in CASES:
        p = params_for(name)
        row = {"tap_1k": largest_tap(p, 1000.0), "tap_500": largest_tap(p, 500.0)}
        try:
            traj = D.load(name)
        except FileNotFoundError:
            traj = None
        if traj is not None:
            row["T"] = float(traj["T"])
            row["swingup_1k"], row["max_x_1k"] = swingup_score(p, traj, 1000.0)
            row["swingup_500"], row["max_x_500"] = swingup_score(p, traj, 500.0)
        out[name] = row
        su = (f"swing-up {row['T']:.2f}s: {row['swingup_1k']}/10 @1kHz (max|x| {row['max_x_1k'] * 1000:.0f}mm), "
              f"{row['swingup_500']}/10 @500Hz") if traj is not None else "no swing-up trajectory"
        print(f"{name}: balance tap {row['tap_1k']:.2f} rad/s @1kHz, {row['tap_500']:.2f} @500Hz | {su}", flush=True)
    (model.SIM_DIR / "results" / "q5_double.json").write_text(json.dumps(out, indent=2))
