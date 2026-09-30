"""Motor choice check (29 Sep): fly the chosen double swing-up plan (planned for the Pololu) on clone-motor rigs,
50 random post-sysid rigs each (double_montecarlo.random_rig), TVLQR + Kalman designed on each motor's model.
Run from sim/:  uv run python clone_motor_double.py [name filter]"""
import math, sys
import numpy as np
sys.path.insert(0, ".")
import double_control as C, double_trajopt as D, double_montecarlo as MC, model

def clone(rpm, stall_a=2.3, i0=0.15):
    w = rpm * 2 * math.pi / 60; R = 12 / stall_a; Ke = (12 - i0 * R) / w
    return dict(motor__R=R, motor__Kt=Ke, motor__Ke=Ke, motor__I_noload=i0)

TRAJ = "30T_30-20_damping150_u60_x30"
cases = {"Pololu 4757": {}, "JGB37-520 1000 rpm": clone(1000), "JGB37-520 1600 rpm": clone(1600)}
only = sys.argv[1:] 
for name, ov in cases.items():
    if only and not any(o in name for o in only): continue
    nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=0.068, **ov)
    design = model.with_params(nom, joint2__damping=nom["joint2"]["damping"] * 1.5)
    traj = D.load(TRAJ); g = C.tvlqr(design, traj, 500.0); kf = C.kalman_gains(design, g)
    rng = np.random.default_rng(1234); ok = 0; n = 50
    for i in range(n):
        p = MC.random_rig(nom, rng, 1.0)
        start = (0.0, math.pi + rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
        ok += bool(C.trial(p, traj, rate_hz=500.0, gains=g, kf=kf, start=start, seed=i)["ok"])
    print(f"{name:22s} double swing-up (Pololu-planned trajectory): {ok}/{n} rigs", flush=True)
