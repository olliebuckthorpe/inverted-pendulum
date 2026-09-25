"""Monte Carlo: fly the Kalman-filtered double swing-up on random 'real rigs' whose parameters are ALL wrong
at once, within the errors expected after system ID (Phase 4 / 8.3). Datasheet sensor noise throughout.

Run:  uv run python double_montecarlo.py [trajectory name] [n rigs] [error scale]
"""

import json
import math
import sys

import numpy as np

import double_control as C
import double_trajopt as D
import model

# Expected ± error (fraction) left after system ID. Masses are weighed; the rest is fitted from recordings.
POST_SYSID = {
    "cart.mass": 0.02, "joint2.mass": 0.02, "pole.mass": 0.02, "pole2.mass": 0.02, "pole.hub_mass": 0.05,
    "motor.Kt": 0.10, "motor.Ke": 0.05, "motor.R": 0.05, "motor.rotor_inertia": 0.15,
    "cart.coulomb_friction": 0.30, "cart.viscous_friction": 0.30,
    "pole.pivot_damping": 0.30, "joint2.damping": 0.30,
}


def random_rig(nom: dict, rng: np.random.Generator, scale: float = 1.0) -> dict:
    ov = {}
    for key, frac in POST_SYSID.items():
        sec, k = key.split(".")
        ov[f"{sec}__{k}"] = nom[sec][k] * (1 + rng.uniform(-1, 1) * frac * scale)
    ov["sim__control_delay"] = rng.uniform(0.001, 0.004)
    return model.with_params(nom, **ov)


def run(traj_name: str = "30T_30-20", n: int = 30, scale: float = 1.0, rate: float = 500.0,
        design_overrides: dict | None = None) -> dict:
    """design_overrides: the model the controller is DESIGNED on, if it differs from the rig's nominal
    (e.g. {"joint2__damping": ×1.3} to plan against extra joint-2 friction). Rigs are drawn around nominal."""
    nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=0.068)
    design = model.with_params(nom, **(design_overrides or {}))
    traj = D.load(traj_name)
    g = C.tvlqr(design, traj, rate)
    kf = C.kalman_gains(design, g)
    rng = np.random.default_rng(1234)
    ok = []
    for i in range(n):
        p = random_rig(nom, rng, scale)
        start = (0.0, math.pi + rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
        ok.append(bool(C.trial(p, traj, rate_hz=rate, gains=g, kf=kf, start=start, seed=i)["ok"]))
    return {"trajectory": traj_name, "rigs": n, "error_scale": scale, "rate_hz": rate, "success": sum(ok)}


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "30T_30-20"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    for scale in ([float(sys.argv[3])] if len(sys.argv) > 3 else [0.5, 1.0, 1.5]):
        r = run(name, n, scale)
        print(f"{name}: post-sysid errors ×{scale}: {r['success']}/{n} random rigs swing up and balance", flush=True)
