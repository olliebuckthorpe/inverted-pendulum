import math, sys, numpy as np, double_control as C, double_trajopt as D, model
def hard_worlds(nom):
    m, c, j2, q, q2 = nom["motor"], nom["cart"], nom["joint2"], nom["pole"], nom["pole2"]
    return {"nominal": {}, "rotor x1.4": {"motor__rotor_inertia": m["rotor_inertia"]*1.4}, "Kt -20%": {"motor__Kt": m["Kt"]*0.8},
            "j2 mass +25%": {"joint2__mass": j2["mass"]*1.25}, "j2 mass -25%": {"joint2__mass": j2["mass"]*0.75},
            "rod2 +20%": {"pole2__mass": q2["mass"]*1.2}, "joint fric x2": {"pole__pivot_damping": q["pivot_damping"]*2, "joint2__damping": j2["damping"]*2}}
def score(traj_name, Q=None, R=None, rate=500.0, seeds=5, kf_W=None):
    nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=0.068)
    traj = D.load(traj_name); g = C.tvlqr(nom, traj, rate, Q, R)
    kf = C.kalman_gains(nom, g) if kf_W is None else C.kalman_gains(nom, g, kf_W)
    out = {}
    for name, ov in hard_worlds(nom).items():
        p = model.with_params(nom, **ov)
        out[name] = sum(C.trial(p, traj, rate_hz=rate, gains=g, kf=kf, seed=s, start=(0, math.pi + np.random.default_rng(s).uniform(-0.02,0.02), 0))["ok"] for s in range(seeds))
    return out
