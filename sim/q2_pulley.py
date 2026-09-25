"""SPEC §11 Q2 (pulley), with Q1 (motor margin) and Q6 (supply current) measured on each pulley.

For 20T and 30T, under several "worlds" (nominal, weaker motor, more friction, rotor inertia ×0.5 / ×2):
  - swing-up: successes out of 10, catch time, peak cart travel
  - supply current: peak, RMS, longest continuous stretch above 5.25 A (where the PSU may start to trip)
  - the same swing-up with the supply capped at 5 A
  - LQR balance robustness: the largest tap (sudden pole spin, rad/s) it recovers from

Run:  uv run python q2_pulley.py     (prints a table, writes plots/q2-*.png and results/q2_pulley.json)
"""

from __future__ import annotations

import json
import math

import numpy as np

import lqr
import model
import swingup
from plant import Plant, wrap

PSU_TRIP = 5.25  # A: 105% of the GST60A12's 5 A rating (Mean Well spec: trips at 105–150%)


def worlds(base: dict) -> dict[str, dict]:
    m, c = base["motor"], base["cart"]
    return {
        "nominal": base,
        "motor -30% torque": model.with_params(base, motor__Kt=m["Kt"] * 0.7),
        "friction +30%": model.with_params(
            base, cart__coulomb_friction=c["coulomb_friction"] * 1.3,
            cart__viscous_friction=c["viscous_friction"] * 1.3, motor__I_noload=m["I_noload"] * 1.3),
        "rotor inertia x0.5": model.with_params(base, motor__rotor_inertia=m["rotor_inertia"] * 0.5),
        "rotor inertia x2": model.with_params(base, motor__rotor_inertia=m["rotor_inertia"] * 2),
    }


def longest_run_above(i_supply: np.ndarray, level: float, dt: float) -> float:
    best = run = 0
    for above in i_supply > level:
        run = run + 1 if above else 0
        best = max(best, run)
    return best * dt


def swingup_stats(p: dict, K: np.ndarray, n: int = 10) -> dict:
    rs = [swingup.swingup_trial(p, seed=s, K=K) for s in range(n)]
    dt = p["sim"]["timestep"]
    i_s = [r["log"]["i_supply"] for r in rs]
    ok = [r for r in rs if r["ok"]]
    return {
        "success": sum(r["ok"] for r in rs),
        "catch_s": float(np.median([r["t_catch"] for r in ok])) if ok else math.nan,
        "max_x_mm": float(max(r["max_x"] for r in rs) * 1000),
        "peak_supply_A": float(max(i.max() for i in i_s)),
        "rms_supply_A": float(np.sqrt(np.mean(np.concatenate(i_s) ** 2))),
        "longest_above_trip_ms": float(max(longest_run_above(i, PSU_TRIP, dt) for i in i_s) * 1000),
        "peak_motor_A": float(max(r["peak_motor_A"] for r in rs)),
    }


def survives_tap(p: dict, K: np.ndarray, kick: float, rate_hz: float = 500.0) -> bool:
    """Balance upright for 1 s, tap the pole (both directions tried), then it must be back within 5° by 4 s."""
    for sign in (1, -1):
        plant = Plant(p, rate_hz=rate_hz)
        plant.reset(th=0.0)
        done = {"k": False}

        def disturb(t, sign=sign):
            if not done["k"] and t >= 1.0:
                done["k"] = True
                return sign * kick
            return 0.0
        log = plant.run(lqr.LQRController(K), 4.0, stop=lqr.fell(60), disturb=disturb)
        if log["t"][-1] < 4.0 - 2 * plant.dt or abs(wrap(log["th"][-1])) > math.radians(5):
            return False
    return True


def largest_tap(p: dict, K: np.ndarray, rate_hz: float = 500.0, hi: float = 20.0) -> float:
    """Binary search for the biggest tap (rad/s of sudden pole spin) that LQR recovers from."""
    lo = 0.0
    if survives_tap(p, K, hi, rate_hz):
        return hi
    for _ in range(12):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if survives_tap(p, K, mid, rate_hz) else (lo, mid)
    return lo


def evaluate(p: dict) -> dict:
    K = lqr.design(p, 500)
    row = swingup_stats(p, K)
    capped = model.with_params(p, supply__current_cap=5.0)
    row["success_capped_5A"] = swingup_stats(capped, K)["success"]
    row["largest_tap_rad_s"] = largest_tap(p, K)
    return row


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    results = {}
    for teeth in (20, 30):
        base = model.load_params(**{"drive.pulley_teeth": teeth})
        for name, p in worlds(base).items():
            results[f"{teeth}T | {name}"] = row = evaluate(p)
            print(f"{teeth}T | {name:20s} swing-up {row['success']:2d}/10 (5 A cap {row['success_capped_5A']:2d}/10) "
                  f"catch {row['catch_s']:.2f}s  max|x| {row['max_x_mm']:.0f}mm  "
                  f"supply peak {row['peak_supply_A']:.1f}A rms {row['rms_supply_A']:.1f}A "
                  f">5.25A for {row['longest_above_trip_ms']:.0f}ms  motor peak {row['peak_motor_A']:.1f}A  "
                  f"largest tap {row['largest_tap_rad_s']:.2f} rad/s", flush=True)

    out = model.SIM_DIR / "results"
    out.mkdir(exist_ok=True)
    (out / "q2_pulley.json").write_text(json.dumps(results, indent=2))

    # Side-by-side nominal swing-ups: cart travel and supply current
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for teeth, col in ((20, "C0"), (30, "C1")):
        p = model.load_params(**{"drive.pulley_teeth": teeth})
        r = swingup.swingup_trial(p, seed=0)
        L, sel = r["log"], r["log"]["t"] < 5
        ax[0].plot(L["t"][sel], np.degrees([wrap(a) for a in L["th"][sel]]), col, label=f"{teeth}T")
        ax[1].plot(L["t"][sel], L["x"][sel] * 1000, col)
        ax[2].plot(L["t"][sel], L["i_supply"][sel], col)
    ax[2].axhline(PSU_TRIP, color="r", ls="--", lw=0.8, label="PSU may trip (5.25 A)")
    ax[0].set_ylabel("θ (deg, 0 = up)"); ax[1].set_ylabel("cart x (mm)"); ax[2].set_ylabel("supply current (A)")
    ax[2].set_xlabel("t (s)"); ax[0].legend(); ax[2].legend(fontsize=8)
    fig.suptitle("Q2: swing-up on 20T vs 30T (nominal datasheet motor, 30 cm rod, 500 Hz)")
    fig.tight_layout()
    fig.savefig(model.SIM_DIR / "plots" / "q2-swingup-20T-vs-30T.png", dpi=90)
