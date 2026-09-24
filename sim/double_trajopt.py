"""Double swing-up by trajectory optimisation (direct collocation), the classical method (Tedrake, ch. 10).

Question: does ANY motor-command sequence take both rods from hanging to upright, within the rig's limits?
We split the swing-up into N short intervals and let the optimiser (IPOPT) choose the state and duty at
every knot, subject to:
  - physics: between knots the state must follow the equations of motion (trapezoidal rule);
  - start: cart centred, both rods hanging still;   end: both rods upright and still, cart centred;
  - duty within ±u_max (margin left for the tracking controller), cart within ±x_max,
    supply current within the PSU limit (SPEC §11 Q6).
It minimises the swing-up time plus a small penalty on duty² (smoother = easier to track).

A converged answer proves a swing-up exists in the model. A failure to converge is NOT proof that none
exists (the optimiser can get stuck), so several starting guesses are tried.

Run:  uv run python double_trajopt.py      (tries both pulleys × the four PLAN 1.6 Q5 layouts)
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import casadi as ca
import numpy as np

import double_eom
import model


@dataclass
class Limits:
    u_max: float = 0.8          # duty
    x_max: float = 0.35         # m
    supply_max: float = 5.0     # A
    T_min: float = 0.8
    T_max: float = 12.0
    smooth: float = 0.2         # weight on ∫duty² dt


def solve(p: dict, lim: Limits | None = None, N: int = 150, T_guess: float = 2.5, a1_end: float = 0.0,
          a2_end: float = 0.0, seed: int = 0, verbose: bool = False, warm: dict | None = None) -> dict:
    lim = lim or Limits()
    f = double_eom.casadi_dynamics(p)
    m = p["motor"]
    V, R, Ke, r = p["supply"]["voltage"], m["R"], m["Ke"], model.pulley_radius(p)

    opti = ca.Opti()
    X = opti.variable(6, N + 1)
    U = opti.variable(1, N + 1)
    T = opti.variable()
    h = T / N

    for k in range(N):  # trapezoidal collocation
        fk, fk1 = f(X[:, k], U[k]), f(X[:, k + 1], U[k + 1])
        opti.subject_to(X[:, k + 1] == X[:, k] + h / 2 * (fk + fk1))
    opti.subject_to(X[:, 0] == ca.DM([0, math.pi, 0, 0, 0, 0]))
    opti.subject_to(X[:, N] == ca.DM([0, a1_end, a2_end, 0, 0, 0]))
    opti.subject_to(opti.bounded(-lim.u_max, U, lim.u_max))
    opti.subject_to(opti.bounded(-lim.x_max, X[0, :], lim.x_max))
    opti.subject_to(opti.bounded(lim.T_min, T, lim.T_max))
    # supply current ≈ duty × (V·duty − Ke·ω)/R  must stay under the PSU limit
    omega = X[3, :] / r
    opti.subject_to(U * (V * U - Ke * omega) / R <= lim.supply_max)

    opti.minimize(T + lim.smooth * h * ca.sumsqr(U))

    # Initial guess: rod 1 rotates up (with some wobble so different seeds explore differently)
    rng = np.random.default_rng(seed)
    s = np.linspace(0, 1, N + 1)
    guess = np.zeros((6, N + 1))
    guess[1] = math.pi + (a1_end - math.pi) * s
    guess[2] = a2_end * s + rng.normal(0, 0.3) * np.sin(math.pi * s)
    guess[0] = 0.1 * rng.normal() * np.sin(2 * math.pi * s)
    opti.set_initial(X, guess)
    opti.set_initial(U, 0.2 * rng.normal(size=N + 1))
    opti.set_initial(T, T_guess)
    if warm is not None:  # start from a previous solution (continuation)
        opti.set_initial(X, warm["X"]); opti.set_initial(U, warm["U"]); opti.set_initial(T, warm["T"])

    opti.solver("ipopt", {"print_time": False}, {"max_iter": 3000, "print_level": 5 if verbose else 0,
                                                  "sb": "yes"})
    try:
        sol = opti.solve()
        ok = True
    except RuntimeError:
        sol = opti.debug
        ok = False
    status = opti.stats()["return_status"]
    Xs, Us, Ts = np.array(sol.value(X)), np.array(sol.value(U)).ravel(), float(sol.value(T))
    i_supply = Us * (V * Us - Ke * Xs[3] / r) / R
    return {"ok": ok, "status": status, "T": Ts, "X": Xs, "U": Us, "t": np.linspace(0, Ts, N + 1),
            "max_x": float(np.abs(Xs[0]).max()), "peak_supply_A": float(i_supply.max()),
            "max_a1d": float(np.abs(Xs[4]).max()), "max_a2d": float(np.abs(Xs[5]).max())}


def best_of(p: dict, tries: int = 6, **kw) -> dict:
    """Try several end-angle choices and starting guesses; keep the fastest converged swing-up."""
    best = None
    ends = [(0.0, 0.0), (2 * math.pi, 0.0), (0.0, 2 * math.pi), (0.0, -2 * math.pi)]
    for i in range(tries):
        a1e, a2e = ends[i % len(ends)]
        r = solve(p, a1_end=a1e, a2_end=a2e, seed=i, T_guess=2.0 + 0.5 * (i % 3), **kw)
        if r["ok"] and (best is None or r["T"] < best["T"]):
            best = r
    return best or {"ok": False}


def continuation(p: dict, final: Limits, steps: int = 12, N: int = 300, a1_end: float = 0.0,
                 a2_end: float = 0.0, verbose: bool = False) -> dict:
    """Solve with a huge motor and long rail, then tighten toward `final`, warm-starting each step.
    Returns the last converged solution and how far the tightening got (1.0 = all the way)."""
    easy = Limits(u_max=20.0, x_max=5.0, supply_max=1e3, T_min=final.T_min, T_max=final.T_max, smooth=final.smooth)
    prev, reached = None, 0.0
    for i in range(steps + 1):
        s = i / steps  # interpolate geometrically: each step tightens by the same ratio
        lim = Limits(u_max=easy.u_max * (final.u_max / easy.u_max) ** s,
                     x_max=easy.x_max * (final.x_max / easy.x_max) ** s,
                     supply_max=easy.supply_max * (final.supply_max / easy.supply_max) ** s,
                     T_min=final.T_min, T_max=final.T_max, smooth=final.smooth)
        r = solve(p, lim, N=N, T_guess=3.0, a1_end=a1_end, a2_end=a2_end, warm=prev)
        if verbose:
            print(f"  step {i:2d}: u_max {lim.u_max:6.2f} x_max {lim.x_max:5.2f} m supply {lim.supply_max:7.1f} A -> "
                  f"{r['status']} T={r['T']:.2f}s", flush=True)
        if not r["ok"]:
            break
        prev, reached = r, s
    out = dict(prev) if prev else {"ok": False}
    out["reached"] = reached
    out["ok"] = reached == 1.0
    return out


LAYOUTS = [(0.30, 0.20), (0.30, 0.30), (0.20, 0.30), (0.34, 0.20)]  # bottom / top, PLAN 1.6 Q5


def layout_params(teeth: int, bottom: float, top: float) -> dict:
    base = model.load_params(**{"drive.pulley_teeth": teeth})
    # rod masses scale with length: aluminium 8 mm tube ≈ 0.067 kg/m, carbon ≈ 0.075 kg/m (0.015 kg / 0.20 m)
    return model.with_params(base, pole__length=bottom, pole__mass=0.067 * bottom,
                             pole2__length=top, pole2__mass=0.075 * top)


def save(r: dict, name: str) -> None:
    d = model.SIM_DIR / "results" / "trajectories"
    d.mkdir(parents=True, exist_ok=True)
    np.savez(d / f"{name}.npz", X=r["X"], U=r["U"], t=r["t"], T=r["T"])


def load(name: str) -> dict:
    z = np.load(model.SIM_DIR / "results" / "trajectories" / f"{name}.npz")
    return {k: z[k] for k in z.files}


if __name__ == "__main__":
    import sys
    cases = [(t, b, u) for t in (30, 20) for b, u in LAYOUTS]
    x_max = Limits().x_max
    suffix = ""
    if len(sys.argv) > 1:  # one case, e.g. "30T_30-20", so cases can run in parallel processes
        teeth, rods = sys.argv[1].split("_")
        cases = [(int(teeth[:-1]), *(int(v) / 100 for v in rods.split("-")))]
    if len(sys.argv) > 2:  # optional tighter rail limit (m), e.g. 0.30, to leave the tracker room
        x_max = float(sys.argv[2])
        suffix = f"_x{int(x_max * 100)}"
    out = {}
    for teeth, bottom, top in cases:
        key = f"{teeth}T_{int(bottom * 100)}-{int(top * 100)}{suffix}"
        r = continuation(layout_params(teeth, bottom, top), Limits(x_max=x_max), steps=6)
        if r["ok"]:
            save(r, key)
            out[key] = {k: r[k] for k in ("T", "max_x", "peak_supply_A", "max_a1d", "max_a2d")}
            print(f"{key}: FOUND swing-up in {r['T']:.2f} s, max|x| {r['max_x'] * 1000:.0f} mm, "
                  f"peak supply {r['peak_supply_A']:.1f} A, peak rod speeds {r['max_a1d']:.0f}/{r['max_a2d']:.0f} rad/s",
                  flush=True)
        else:
            out[key] = {"reached": r["reached"]}
            print(f"{key}: not found (tightening reached {r['reached'] * 100:.0f}% of the way)", flush=True)
        (model.SIM_DIR / "results").mkdir(exist_ok=True)
        (model.SIM_DIR / "results" / f"double_trajopt_{key}.json").write_text(json.dumps(out[key], indent=2))
