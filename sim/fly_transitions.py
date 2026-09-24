"""Fly planned transitions (results/trajectories/T_*.npz) with the Kalman-filtered tracker, on the nominal rig
and on random post-system-ID rigs. Datasheet sensor noise and the 5 A limit throughout.
Usage: uv run python fly_transitions.py [T_UU-UD ...]   (default: every saved transition)"""
import math
import sys

import numpy as np

import double_control as C
import double_montecarlo as M
import double_trajopt as D
import model

nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=0.068)
design = model.with_params(nom, joint2__damping=nom["joint2"]["damping"] * 1.5)
names = sys.argv[1:] or sorted(f.stem for f in (model.SIM_DIR / "results" / "trajectories").glob("T_*.npz"))

for name in names:
    traj = D.load(name)
    g = C.tvlqr(design, traj, 500.0)
    kf = C.kalman_gains(design, g)
    start = (0.0, float(traj["X"][1, 0]), float(traj["X"][2, 0]))
    nominal = sum(C.trial(nom, traj, rate_hz=500.0, gains=g, kf=kf, seed=s,
                          start=(0.0, start[1] + np.random.default_rng(s).uniform(-0.01, 0.01), start[2]))["ok"] for s in range(5))
    rng = np.random.default_rng(99)
    rigs = sum(C.trial(M.random_rig(nom, rng), traj, rate_hz=500.0, gains=g, kf=kf, seed=i, start=start)["ok"] for i in range(20))
    print(f"{name}: {float(traj['T']):.2f} s | nominal rig {nominal}/5 | random post-sysid rigs {rigs}/20", flush=True)
