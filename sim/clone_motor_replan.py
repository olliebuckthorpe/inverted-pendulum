"""Re-plan the chosen double swing-up (30T, 30/20 rods, joint-2 friction x1.5 margin) for a clone motor.
Usage (from sim/): uv run python clone_motor_replan.py <rpm> <u_max>; then 50 random post-sysid rigs fly it.
29 Sep: 1000 rpm u 0.6 / 1600 rpm u 0.6 / 1600 rpm u 0.8 -> no feasible plan; 1000 rpm u 0.8 -> 4/50."""
import math, sys, time
import numpy as np
sys.path.insert(0, ".")
import double_control as C, double_trajopt as D, double_montecarlo as MC, model

rpm, u_max = int(sys.argv[1]), float(sys.argv[2])
w = rpm * 2 * math.pi / 60; R = 12 / 2.3; Ke = (12 - 0.15 * R) / w
ov = dict(motor__R=R, motor__Kt=Ke, motor__Ke=Ke, motor__I_noload=0.15)
nom = model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0, sensing__angle_noise_deg=0.068, **ov)
design = model.with_params(nom, joint2__damping=nom["joint2"]["damping"] * 1.5)
t0 = time.time()
r = D.continuation(design, D.Limits(u_max=u_max, x_max=0.30), steps=6, verbose=True)
name = f"clone{rpm}_u{int(u_max*100)}"
print(f"{name}: plan {'FOUND T=%.2f s' % r['T'] if r['ok'] else 'NOT FOUND (reached %.0f%% of the tightening)' % (100*r['reached'])} in {(time.time()-t0)/60:.0f} min", flush=True)
if not r["ok"]:
    sys.exit()
D.save(r, name)
traj = D.load(name); g = C.tvlqr(design, traj, 500.0); kf = C.kalman_gains(design, g)
rng = np.random.default_rng(1234); ok = 0
for i in range(50):
    p = MC.random_rig(nom, rng, 1.0)
    start = (0.0, math.pi + rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02))
    ok += bool(C.trial(p, traj, rate_hz=500.0, gains=g, kf=kf, start=start, seed=i)["ok"])
print(f"{name}: re-planned double swing-up flies on {ok}/50 random rigs", flush=True)
