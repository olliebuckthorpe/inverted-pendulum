"""System identification (PLAN Phase 4): record the rig, fit the sim to it, check the fit.

  uv run python sysid.py record PORT all          # 4.1 cart tests (SYSID mode) -> analysis/data/sysid/<test>.csv
  uv run python sysid.py swing PORT --deg 30      # 4.1 free swing, motor braked -> analysis/data/sysid/swing30.csv
  uv run python sysid.py fit [--write]            # 4.2 fit -> analysis/sysid-fit.md (and sim/params.yaml with --write)
  uv run python sysid.py validate                 # 4.3 sim vs real overlays -> analysis/plots/, PASS/FAIL
  uv run python sysid.py rehearse                 # the whole pipeline on synthetic recordings from a known rig

What is fitted, and what can't be:
  - Pendulum (free swing, cart held by the braked motor): effective rod length (sets the period), pivot
    viscous damping and dry friction (set the decay). Weigh the rod and measure its length first.
  - Cart (voltage steps, chirp, slow triangle, rod hanging free): motor Kt and Ke, cart dry and viscous
    friction, and the control delay. With only the duty and the encoder, the recordings pin RATIOS to the moving
    mass (motor push, back-EMF drag and friction per kg), not the parts: a rehearsal fitting Kt and rotor inertia
    together landed +21 % / +26 % off with the motion still matched to 0.5 mm, and the delay 1 ms off. So the
    rotor inertia stays at its prior, the cart is weighed, R is measured with a multimeter (params.yaml
    motor.R), and the report gives the ratios, which are what the sim's motion depends on.
  - Gearbox play: NOT identifiable. The only cart sensor is the motor encoder, which sits before the play.
    It keeps its prior (1°) and the AI's randomisation range (0–2×).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import struct
import time
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

import model
import motor
from plant import Plant

DATA = model.SIM_DIR.parent / "analysis" / "data" / "sysid"
TRAVEL = 0.40          # m each way (params cart.travel): the 4.3 criterion is 10 % of this
SLEW = 0.25            # the firmware's slew limit per 2 ms cycle (control.h SafetyParams)
CTRL_HZ = 500.0

# name: (type, amp, f0, f1, seconds). Sized in the sim to stay within about ±150 mm of the start.
TESTS = {
    "steps25": (0, 0.25, 1.0, 0.0, 8.0),       # ±25 % duty, 1 s steps
    "steps50": (0, 0.50, 2.5, 0.0, 8.0),       # ±50 %, 0.4 s steps
    "steps75": (0, 0.75, 4.0, 0.0, 8.0),       # ±75 %, 0.25 s steps
    "chirp": (1, 0.30, 0.5, 20.0, 20.0),       # 30 % sine sweeping 0.5 -> 20 Hz
    "triangle": (2, 0.15, 0.5, 0.0, 12.0),     # slow ±15 % ramps: the friction test
}
CART_FIT = ("motor.Kt", "motor.Ke", "cart.coulomb_friction", "cart.viscous_friction")
PRIOR_SIGMA = math.log(3.0)                    # weak prior: each parameter within ×3 of params.yaml


# ---- the excitation, identical to firmware/core/control.h ctl::sysid_duty (parity-tested) ----------------------
def excitation(kind: int, amp: float, f0: float, f1: float, T: float, t: float) -> float:
    f32 = np.float32
    amp, f0, f1, T, t = (f32(v) for v in (amp, f0, f1, T, t))
    if t >= T:
        return 0.0
    if kind == 0:
        per = f32(1) / f0 if f0 > 0 else f32(1)
        return float(amp if np.fmod(t + f32(0.5) * per, f32(2) * per) < per else -amp)
    if kind == 1:
        k = (f1 - f0) / T
        return float(amp * np.sin(f32(2) * f32(math.pi) * (f0 * t + f32(0.5) * k * t * t)))
    ph = np.fmod(t * f0, f32(1))
    return float(amp * (f32(4) * ph - f32(1) if ph < f32(0.5) else f32(3) - f32(4) * ph))


# ---- recordings --------------------------------------------------------------------------------------------------
def save(path: Path, rec: dict, meta: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        f.write("# " + json.dumps(meta) + "\n")
        w = csv.writer(f)
        w.writerow(("t", "x", "th", "duty"))
        for row in zip(rec["t"], rec["x"], rec["th"], rec["duty"]):
            w.writerow([f"{v:.6f}" for v in row])


def load(path: Path) -> tuple[dict, dict]:
    lines = path.read_text().splitlines()
    meta = json.loads(lines[0][2:])
    rows = list(csv.DictReader(lines[1:]))
    return {k: np.array([float(r[k]) for r in rows]) for k in ("t", "x", "th", "duty")}, meta


# ---- simulation of a recording ----------------------------------------------------------------------------------
def replay(p: dict, rec: dict) -> np.ndarray:
    """Drive the sim with the duty the Pico actually applied (after its supply cap and slew limit) and return the
    sim's cart position at the recording's times. Rigid drive: the gearbox play isn't identifiable (see top)."""
    q = model.with_params(p, supply__current_cap=None)
    t, duty = rec["t"] - rec["t"][0], rec["duty"]
    pl = Plant(q, rate_hz=CTRL_HZ)
    pl.reset(x=rec["x"][0], th=rec["th"][0])
    log = pl.run(lambda tt, _obs: duty[min(np.searchsorted(t, tt, side="right") - 1, len(duty) - 1)], t[-1] + 0.002)
    return np.interp(t, log["t"], log["x"])


def swing_model(p: dict, t: np.ndarray, th0: float, thd0: float) -> np.ndarray:
    """Free swing of rod 1 with the cart held still. θ = 0 is UPRIGHT (the project's convention), so gravity
    pushes it away from 0: J·θ'' = +m·g·(L/2)·sin θ − b·θ' − c·sign(θ'); hanging (θ = π) is the stable point."""
    q = p["pole"]
    J = model.pole_inertia_about_pivot(p)
    mgl, b, c = q["mass"] * 9.81 * q["length"] / 2, q["pivot_damping"], q.get("pivot_coulomb", 0.0)
    f = lambda _t, s: (s[1], (mgl * math.sin(s[0]) - b * s[1] - c * math.tanh(s[1] / 0.01)) / J)
    return solve_ivp(f, (t[0], t[-1]), (th0, thd0), t_eval=t, rtol=1e-8, atol=1e-10, max_step=0.005).y[0]


def swing_start(rec: dict) -> tuple[np.ndarray, float, float]:
    """Start the fit at the first moment after release (rod moving back through the middle of its first
    half-swing), so the hand's push isn't part of the data."""
    th = np.unwrap(rec["th"])
    t = rec["t"] - rec["t"][0]
    k = 15                                                 # 30 ms moving average: sensor noise alone differentiates
    ths = np.convolve(th, np.ones(k) / k, mode="same")     # to ~±0.4 rad/s at 500 Hz
    thd = np.gradient(ths, t)
    i0 = int(np.argmax(np.abs(thd[k:-k]) > 0.5)) + k + 50  # 100 ms after it starts moving (hand clear)
    return t[i0:] - t[i0], float(ths[i0]), float(thd[i0])


def period(t: np.ndarray, th: np.ndarray, h: float = math.radians(1.0)) -> float:
    """Mean period from the upward zero crossings of (θ − hanging), with hysteresis: a crossing only counts once
    the angle has been more than h below hanging since the last one, so sensor noise (and the tiny swings at the
    end of a decay) can't add crossings."""
    d = np.unwrap(th) - math.pi * np.round(np.median(np.unwrap(th)) / math.pi)
    tz, armed = [], False
    for i in range(1, len(d)):
        if d[i] < -h:
            armed = True
        elif armed and d[i - 1] < 0 <= d[i]:
            tz.append(t[i - 1] - d[i - 1] * (t[i] - t[i - 1]) / (d[i] - d[i - 1]))
            armed = False
    return float(np.mean(np.diff(tz))) if len(tz) > 2 else float("nan")


# ---- fitting ------------------------------------------------------------------------------------------------------
def _get(p: dict, dotted: str) -> float:
    sec, k = dotted.split(".")
    return p[sec][k]


def _scaled(p0: dict, names, logm) -> dict:
    return model.with_params(p0, **{n.replace(".", "__"): _get(p0, n) * math.exp(v) for n, v in zip(names, logm)})


def fit_swing(p0: dict, recs: list[dict]) -> tuple[dict, dict]:
    names = ("pole.length", "pole.pivot_damping")
    p0 = model.with_params(p0, pole__pivot_coulomb=max(p0["pole"].get("pivot_coulomb", 0.0), 1e-5))
    names = names + ("pole.pivot_coulomb",)
    starts = [swing_start(r) for r in recs]
    n = len(names)

    def res(v):
        p = _scaled(p0, names, v[:n])
        # rod mass is weighed: keep it fixed while the effective length moves
        p = model.with_params(p, pole__mass=p0["pole"]["mass"])
        # each swing's starting angle and speed are fitted too (small corrections to the smoothed estimates)
        r = [(swing_model(p, t, a + v[n + 2 * i], w + v[n + 2 * i + 1]) - np.unwrap(rec["th"])[-len(t):]) / math.radians(0.1)
             for i, ((t, a, w), rec) in enumerate(zip(starts, recs))]
        return np.concatenate(r + [np.asarray(v[:n]) / PRIOR_SIGMA])

    sol = least_squares(res, np.zeros(n + 2 * len(recs)), diff_step=1e-3)
    p = model.with_params(_scaled(p0, names, sol.x[:n]), pole__mass=p0["pole"]["mass"])
    return p, {n: _get(p, n) for n in names}


def ratios(p: dict) -> dict:
    """What the cart recordings actually pin down (per kg of moving mass, incl. the rotor reflected through the
    gearbox and pulley): motor push per unit duty, back-EMF drag per m/s, and dry friction."""
    M = p["cart"]["mass"] + p["pole"]["hub_mass"] + p["pole"]["mass"] + model.reflected_rotor_mass(p)
    return {"push_per_kg (m/s² at duty 1)": model.motor_gain(p) / M,
            "drag_per_kg (1/s)": (model.motor_damping(p) + p["cart"]["viscous_friction"]) / M,
            "dry_friction_per_kg (m/s²)": (p["cart"]["coulomb_friction"] + model.gearbox_drag(p)) / M}


def fit_cart(p0: dict, recs: list[dict], delays=(0.001, 0.002, 0.003, 0.004, 0.005, 0.006)) -> tuple[dict, dict]:
    """Least squares on the cart position of every cart test, at each candidate delay (the sim's delay is a whole
    number of 1 ms physics steps, so it's searched, not differentiated), warm-started; keep the best."""
    best, v = None, np.zeros(len(CART_FIT))
    for d in delays:
        pd = model.with_params(p0, sim__control_delay=d)

        def res(x, pd=pd):
            p = _scaled(pd, CART_FIT, x)
            r = [(replay(p, rec) - rec["x"]) / 0.002 for rec in recs]
            return np.concatenate(r + [np.asarray(x) / PRIOR_SIGMA])

        sol = least_squares(res, v, diff_step=1e-3, max_nfev=25)
        v = sol.x
        if best is None or sol.cost < best[0]:
            best = (sol.cost, d, sol)
    cost, d, sol = best
    p = _scaled(model.with_params(p0, sim__control_delay=d), CART_FIT, sol.x)
    out = {n: _get(p, n) for n in CART_FIT}
    out["sim.control_delay"] = d
    out["ratios"] = ratios(p)
    return p, out


# ---- 4.3 validation ------------------------------------------------------------------------------------------------
def validate(p: dict, cart: dict[str, dict], swings: list[dict]) -> dict:
    out = {}
    for name, rec in cart.items():
        t = rec["t"] - rec["t"][0]
        w = t <= 5.0
        err = np.abs(replay(p, rec) - rec["x"])[w].max()
        out[name] = {"max_err_mm": 1000 * float(err), "pass": bool(err < 0.10 * TRAVEL)}
    for i, rec in enumerate(swings):
        t, a, w = swing_start(rec)
        real = period(t, np.unwrap(rec["th"])[-len(t):])
        sim = period(t, swing_model(p, t, a, w))
        out[f"swing{i}"] = {"period_real_s": real, "period_sim_s": sim,
                            "err_pct": 100 * abs(sim - real) / real, "pass": bool(abs(sim - real) / real < 0.03)}
    return out


def plots(p: dict, cart: dict[str, dict], swings: list[dict], outdir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    outdir.mkdir(parents=True, exist_ok=True)
    for name, rec in cart.items():
        t = rec["t"] - rec["t"][0]
        fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        ax[0].plot(t, rec["x"] * 1000, label="real", lw=1.2)
        ax[0].plot(t, replay(p, rec) * 1000, label="fitted sim", lw=1.0, ls="--")
        ax[0].set_ylabel("cart (mm)"); ax[0].legend()
        ax[1].plot(t, rec["duty"], lw=0.8); ax[1].set_ylabel("duty"); ax[1].set_xlabel("time (s)")
        fig.suptitle(f"System ID: {name}"); fig.tight_layout(); fig.savefig(outdir / f"sysid-{name}.png", dpi=120)
        plt.close(fig)
    for i, rec in enumerate(swings):
        t, a, w = swing_start(rec)
        fig, ax = plt.subplots(figsize=(9, 3.5))
        ax.plot(t, np.degrees(np.unwrap(rec["th"])[-len(t):]), label="real", lw=1.2)
        ax.plot(t, np.degrees(swing_model(p, t, a, w)), label="fitted sim", lw=1.0, ls="--")
        ax.set_xlabel("time (s)"); ax.set_ylabel("rod angle (deg)"); ax.legend()
        fig.suptitle(f"System ID: free swing {i}"); fig.tight_layout(); fig.savefig(outdir / f"sysid-swing{i}.png", dpi=120)
        plt.close(fig)


# ---- synthetic recordings (rehearsal and tests) -----------------------------------------------------------------------
TRUE_RIG = {"motor__Kt": 0.9, "motor__Ke": 1.1, "motor__rotor_inertia": 1.6, "cart__coulomb_friction": 1.8,
            "cart__viscous_friction": 0.6, "pole__pivot_damping": 3.0, "pole__length": 0.90}


def true_rig(p: dict) -> dict:
    """A 'real' rig that differs from params.yaml in every fitted parameter (multipliers above) and delay 3 ms."""
    ov = {}
    for k, m in TRUE_RIG.items():
        sec, name = k.split("__")
        ov[k] = p[sec][name] * m
    ov["pole__pivot_coulomb"] = 2e-4
    ov["sim__control_delay"] = 0.003
    return model.with_params(p, **ov)


def synth_cart(p_true: dict, name: str, seed: int = 0) -> dict:
    """What the Pico would log: its excitation through its supply cap and slew limit, the plant, the encoder's
    quantisation; `duty` is the applied (post-cap, post-slew) value, as in real telemetry."""
    kind, amp, f0, f1, T = TESTS[name]
    pl = Plant(model.with_params(p_true, supply__current_cap=None), rate_hz=CTRL_HZ, seed=seed)
    pl.reset(x=0.0, th=math.pi)
    last = [0.0]

    def ctrl(t, obs):
        u = motor.cap_duty_for_supply(p_true, excitation(kind, amp, f0, f1, T, t), obs[1], 4.5)
        u = min(max(u, last[0] - SLEW), last[0] + SLEW)
        last[0] = u
        return u

    log = pl.run(ctrl, T + 0.5)
    k = np.arange(0, len(log["t"]), round(1 / (CTRL_HZ * pl.dt)))
    step = 2 * math.pi * model.pulley_radius(p_true) / p_true["sensing"]["cart_counts_per_rev"]
    duty_applied = np.array([log["duty"][min(i + pl.delay_steps, len(log["duty"]) - 1)] for i in k])
    return {"t": log["t"][k], "x": np.round(log["x"][k] / step) * step, "th": log["th"][k], "duty": duty_applied}


def synth_swing(p_true: dict, deg: float, seed: int = 0) -> dict:
    t = np.arange(0, 20, 1 / CTRL_HZ)
    rng = np.random.default_rng(seed)
    hold = np.full(100, math.pi + math.radians(deg))           # held at the release angle for 0.2 s
    th = swing_model(p_true, t, math.pi + math.radians(deg), 0.0)
    th = np.concatenate([hold, th]) + rng.normal(0, math.radians(0.068), len(t) + 100)
    step = 2 * math.pi / 2 ** 14
    tt = np.arange(len(th)) / CTRL_HZ
    return {"t": tt, "x": np.zeros_like(tt), "th": np.round(th / step) * step, "duty": np.zeros_like(tt)}


# ---- hardware -----------------------------------------------------------------------------------------------------------
def record_test(port: str, name: str) -> Path:
    import bringup
    import link
    kind, amp, f0, f1, T = TESTS[name]
    L = link.Link(port)
    bringup.configure(L)
    for pid, v in ((70, kind), (71, amp), (72, f0), (73, f1), (74, T)):
        L.send(link.PARAM, struct.pack("<Hf", pid, float(v)))
    time.sleep(0.2)
    L.send(link.SET_MODE, bytes([link.MODES["SYSID"]]))
    rows = [r for r in bringup.telemetry(L, T + 1.0)]
    L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
    faults = sorted({f for r in rows for f in link.decode_faults(r["faults"])[0]})
    rec = {"t": np.array([r["t_us"] for r in rows]) * 1e-6, "x": np.array([r["x"] for r in rows]),
           "th": np.array([r["th"] for r in rows]), "duty": np.array([r["duty"] for r in rows])}
    path = DATA / f"{name}.csv"
    save(path, rec, {"test": name, "spec": TESTS[name], "date": str(dt.datetime.now()), "faults": faults})
    print(f"{name}: {len(rows)} frames, cart {rec['x'].min() * 1000:.0f}..{rec['x'].max() * 1000:.0f} mm, "
          f"faults {faults or 'none'} -> {path.relative_to(model.SIM_DIR.parent)}")
    if faults:
        print("  ⚠️ a fault cut the test short: re-centre the cart and record it again")
    return path


def record_swing(port: str, deg: float, seconds: float) -> Path:
    import bringup
    import link
    L = link.Link(port)
    bringup.configure(L)
    input(f"Cart mid-rail (motor braked in IDLE). Hold the rod at ~{deg:.0f}° from hanging; press Enter, then let go "
          f"cleanly (no push) within 2 s… ")
    rows = list(bringup.telemetry(L, seconds))
    rec = {"t": np.array([r["t_us"] for r in rows]) * 1e-6, "x": np.array([r["x"] for r in rows]),
           "th": np.array([r["th"] for r in rows]), "duty": np.array([r["duty"] for r in rows])}
    path = DATA / f"swing{deg:.0f}.csv"
    save(path, rec, {"test": f"swing{deg:.0f}", "date": str(dt.datetime.now())})
    print(f"{len(rows)} frames -> {path.relative_to(model.SIM_DIR.parent)}")
    return path


# ---- the whole pipeline --------------------------------------------------------------------------------------------------
def run_fit(p0: dict, cart: dict[str, dict], swings: list[dict]) -> tuple[dict, dict]:
    p1, sw = fit_swing(p0, swings) if swings else (p0, {})
    # the cart tests have the rod hanging free, so fit the cart with the pendulum already fitted
    p2, ct = fit_cart(p1, list(cart.values()))
    return p2, {"swing": sw, "cart": ct}


def report(p0: dict, p: dict, fit: dict, val: dict, where: str) -> str:
    rows = [f"| {k} | {_get(p0, k):.4g} | {_get(p, k):.4g} | ×{_get(p, k) / _get(p0, k):.2f} |"
            for k in ("pole.length", "pole.pivot_damping", "pole.pivot_coulomb") + CART_FIT + ("sim.control_delay",)
            if _get(p0, k)]
    r0, r1 = ratios(p0), fit["cart"]["ratios"]
    return "\n".join([
        f"\n## {dt.date.today()}: fit from {where}\n",
        "| parameter | before | fitted | change |", "|---|---|---|---|", *rows, "",
        "What the cart recordings pin down (the sim's motion depends on these, not on Kt or the rotor inertia alone):\n",
        "| ratio | before | fitted |", "|---|---|---|",
        *[f"| {k} | {r0[k]:.3g} | {r1[k]:.3g} |" for k in r1], "",
        "| check (PLAN 4.3) | result | pass |", "|---|---|---|",
        *[f"| {k} | " + (f"max cart error {v['max_err_mm']:.1f} mm (limit {100 * TRAVEL:.0f} mm)" if "max_err_mm" in v
                         else f"period sim {v['period_sim_s']:.4f} s vs real {v['period_real_s']:.4f} s ({v['err_pct']:.2f} %, limit 3 %)")
          + f" | {'✅' if v['pass'] else '❌'} |" for k, v in val.items()],
        "",
    ])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record"); r.add_argument("port"); r.add_argument("test", choices=[*TESTS, "all"])
    s = sub.add_parser("swing"); s.add_argument("port"); s.add_argument("--deg", type=float, default=30.0)
    s.add_argument("--seconds", type=float, default=20.0)
    f = sub.add_parser("fit"); f.add_argument("--write", action="store_true")
    sub.add_parser("validate")
    sub.add_parser("rehearse")
    a = ap.parse_args()
    p0 = model.load_params()
    if a.cmd == "record":
        for name in (TESTS if a.test == "all" else [a.test]):
            input(f"{name}: cart mid-rail, rod hanging still, hands clear. Enter to start… ")
            record_test(a.port, name)
        return
    if a.cmd == "swing":
        record_swing(a.port, a.deg, a.seconds)
        return
    if a.cmd == "rehearse":
        pt = true_rig(p0)
        cart = {n: synth_cart(pt, n) for n in TESTS}
        swings = [synth_swing(pt, 30), synth_swing(pt, 90, seed=1)]
        where = "SYNTHETIC recordings of a known rig (rehearsal)"
    else:
        cart = {n: load(DATA / f"{n}.csv")[0] for n in TESTS if (DATA / f"{n}.csv").exists()}
        swings = [load(pth)[0] for pth in sorted(DATA.glob("swing*.csv"))]
        where = f"{len(cart)} cart tests + {len(swings)} free swings in analysis/data/sysid/"
        if not cart:
            raise SystemExit("no recordings in analysis/data/sysid/: run `sysid.py record` first")
    if a.cmd == "validate":
        val = validate(p0, cart, swings)
        plots(p0, cart, swings, model.SIM_DIR.parent / "analysis" / "plots")
        for k, v in val.items():
            print(k, v)
        return
    p, fit = run_fit(p0, cart, swings)
    val = validate(p, cart, swings)
    text = report(p0, p, fit, val, where)
    print(text)
    if a.cmd == "rehearse":
        truth = true_rig(p0)
        for k in ("pole.length", "pole.pivot_damping") + CART_FIT + ("sim.control_delay",):
            print(f"  truth {k}: {_get(truth, k):.4g}  fitted {_get(p, k):.4g}")
        for k, v in ratios(truth).items():
            print(f"  truth {k}: {v:.4g}  fitted {fit['cart']['ratios'][k]:.4g}")
        return
    out = model.SIM_DIR.parent / "analysis" / "sysid-fit.md"
    with out.open("a") as fh:
        fh.write(text)
    plots(p, cart, swings, model.SIM_DIR.parent / "analysis" / "plots")
    if a.write:
        import yaml
        doc = yaml.safe_load(model.PARAMS_PATH.read_text())
        for k in ("pole.length", "pole.pivot_damping", "pole.pivot_coulomb") + CART_FIT + ("sim.control_delay",):
            sec, name = k.split(".")
            doc[sec][name] = float(_get(p, k))
        print("⚠️ --write rewrites params.yaml without its comments; the git diff shows exactly what changed")
        model.PARAMS_PATH.write_text(yaml.safe_dump(doc, sort_keys=False))


if __name__ == "__main__":
    main()
