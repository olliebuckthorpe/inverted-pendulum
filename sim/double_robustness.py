"""Double swing-up robustness: controller designed on the nominal model, flown on 'wrong' rigs.
Compares plain differencing with the Kalman filter, at the AS5047P datasheet noise unless stated.

Run:  uv run python double_robustness.py [rate_hz]
"""

import json
import math
import sys

import numpy as np

import double_control as C
import double_trajopt as D
import model

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 500.0
NOISE = 0.068  # AS5047P datasheet RMS noise (degrees)

nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=NOISE)
traj = D.load("30T_30-20")
g = C.tvlqr(nom, traj, RATE)
kf = C.kalman_gains(nom, g)
m, c, j2, q, q2 = nom["motor"], nom["cart"], nom["joint2"], nom["pole"], nom["pole2"]

WORLDS = {
    "nominal (0.068° noise)": {},
    "noise 0.10°": {"sensing__angle_noise_deg": 0.10},
    "noise 0.15°": {"sensing__angle_noise_deg": 0.15},
    "cart mass ±20% (+)": {"cart__mass": c["mass"] * 1.2},
    "cart mass ±20% (−)": {"cart__mass": c["mass"] * 0.8},
    "rotor inertia ×1.4": {"motor__rotor_inertia": m["rotor_inertia"] * 1.4},
    "rotor inertia ×0.7": {"motor__rotor_inertia": m["rotor_inertia"] * 0.7},
    "motor Kt −10%": {"motor__Kt": m["Kt"] * 0.9},
    "motor Kt −20%": {"motor__Kt": m["Kt"] * 0.8},
    "rail friction ×2": {"cart__coulomb_friction": c["coulomb_friction"] * 2, "cart__viscous_friction": c["viscous_friction"] * 2},
    "joint-2 mass +10%": {"joint2__mass": j2["mass"] * 1.1},
    "joint-2 mass −10%": {"joint2__mass": j2["mass"] * 0.9},
    "joint-2 mass +25%": {"joint2__mass": j2["mass"] * 1.25},
    "joint-2 mass −25%": {"joint2__mass": j2["mass"] * 0.75},
    "top-rod mass +10%": {"pole2__mass": q2["mass"] * 1.1},
    "top-rod mass +20%": {"pole2__mass": q2["mass"] * 1.2},
    "joint friction ×2": {"pole__pivot_damping": q["pivot_damping"] * 2, "joint2__damping": j2["damping"] * 2},
    "joint friction ×5": {"pole__pivot_damping": q["pivot_damping"] * 5, "joint2__damping": j2["damping"] * 5},
    "delay 4 ms": {"sim__control_delay": 0.004},
}

if __name__ == "__main__":
    out = {}
    print(f"{RATE:.0f} Hz, 10 randomised starts each: differencing | Kalman filter")
    for name, ov in WORLDS.items():
        p = model.with_params(nom, **ov)
        row = {}
        for label, k in (("diff", None), ("kf", kf)):
            ok = 0
            for seed in range(10):
                rng = np.random.default_rng(seed)
                start = (rng.uniform(-0.01, 0.01), math.pi + rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
                ok += C.trial(p, traj, rate_hz=RATE, gains=g, kf=k, start=start, seed=seed)["ok"]
            row[label] = ok
        out[name] = row
        print(f"  {name:24s} {row['diff']:2d}/10 | {row['kf']:2d}/10", flush=True)
    (model.SIM_DIR / "results" / f"double_robustness_{int(RATE)}Hz.json").write_text(json.dumps(out, indent=2))
