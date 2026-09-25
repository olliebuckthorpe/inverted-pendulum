"""Energy-shaping swing-up + hand-over to LQR (PLAN 1.5; Tedrake, Underactuated Robotics, cart-pole chapter).

Pole energy relative to "balanced at the top":   E = ½·J·θ_dot² + m·g·l·(cos θ − 1)
  0 at the top at rest, −2·m·g·l hanging at rest.
Cart acceleration a changes it at:               dE/dt = −m·l·a·θ_dot·cos θ
Choosing a = k_E·(E − E*)·θ_dot·cos θ gives      dE/dt = −m·l·k_E·(E − E*)·(θ_dot·cos θ)²
which always pushes E toward the target E*. E* is set a little above 0 (the upright energy): aiming
exactly at 0 stalls ~30° short, because the pumping fades to nothing just as friction takes its share. A weak centring term keeps the cart on the rail.
The wanted acceleration becomes a duty cycle through the motor model (force = mass × accel + drag).

Run:  uv run python swingup.py      (PLAN 1.5 check: 10 randomised swing-ups, saves a plot)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import lqr
import model
from plant import Plant, wrap

IDLE, SWING, BALANCE = 0, 2, 1


@dataclass
class SwingParams:
    k_energy: float = 60.0        # (m/s²) per (J·rad/s): how hard to pump
    a_max: float = 6.0            # m/s², cap on the pumping acceleration
    k_x: float = 8.0              # centring: (m/s²) per m
    k_v: float = 4.0              # centring: (m/s²) per (m/s)
    nudge: float = 0.4            # rad/s: stand-in for θ_dot·cos θ when starting from rest
    target_margin: float = 0.10   # pump toward E = +margin·2mgl, not 0, so friction can't leave it short
    catch_deg: float = 20.0       # hand over to LQR inside this angle ...
    catch_energy: float = 0.25    # ... and when |E| < this fraction of 2mgl
    drop_deg: float = 35.0        # hand back to swing-up if LQR lets it fall past this


class EnergySwingUp:
    """Classical controller: swing-up by energy pumping, then LQR. `mode` is logged."""

    def __init__(self, p: dict, K: np.ndarray, sp: SwingParams | None = None):
        self.sp = sp or SwingParams()
        q, c = p["pole"], p["cart"]
        self.m, self.l = q["mass"], q["length"] / 2
        self.J = model.pole_inertia_about_pivot(p)
        self.E_bottom = -2 * self.m * 9.81 * self.l
        # Motor-model inverse: duty = (M_total·a + drag) / G
        self.M_total = c["mass"] + q["hub_mass"] + q["mass"] + model.reflected_rotor_mass(p)
        self.G = model.motor_gain(p)
        self.D = model.motor_damping(p) + c["viscous_friction"]
        self.F_coulomb = c["coulomb_friction"] + model.gearbox_drag(p)
        self.lqr = lqr.LQRController(K)
        self.mode = SWING

    def energy(self, th: float, thd: float) -> float:
        return 0.5 * self.J * thd * thd + self.m * 9.81 * self.l * (math.cos(th) - 1)

    def __call__(self, t: float, obs: np.ndarray) -> float:
        x, xd, th, thd = obs
        sp = self.sp
        E = self.energy(th, thd)
        near_top = abs(th) < math.radians(sp.catch_deg)
        if self.mode == SWING and near_top and abs(E) < sp.catch_energy * abs(self.E_bottom):
            self.mode = BALANCE
        elif self.mode == BALANCE and abs(th) > math.radians(sp.drop_deg):
            self.mode = SWING
        if self.mode == BALANCE:
            return self.lqr(t, obs)

        s = thd * math.cos(th)
        if abs(s) < 1e-3 and E < 0.9 * self.E_bottom:  # hanging still: give it a start
            s = sp.nudge
        E_err = E - sp.target_margin * abs(self.E_bottom)
        a = float(np.clip(sp.k_energy * E_err * s, -sp.a_max, sp.a_max))
        a += -sp.k_x * x - sp.k_v * xd
        force = self.M_total * a + self.D * xd + self.F_coulomb * np.sign(xd)
        return force / self.G


def swingup_trial(p: dict, rate_hz: float = 500.0, seed: int = 0, sp: SwingParams | None = None,
                  compliant: bool = False, seconds: float = 10.0, hold: float = 2.0, K=None) -> dict:
    """One trial from hanging (with a small random offset). Success = balancing (LQR mode, |θ| < 10°)
    for the final `hold` seconds, having caught it by seconds − hold, with the cart never at the rail end."""
    rng = np.random.default_rng(seed)
    K = lqr.design(p, rate_hz) if K is None else K
    plant = Plant(p, rate_hz=rate_hz, compliant=compliant, seed=seed)
    plant.reset(x=rng.uniform(-0.02, 0.02), th=math.pi + rng.uniform(-0.03, 0.03), thd=rng.uniform(-0.1, 0.1))
    ctrl = EnergySwingUp(p, K, sp)
    log = plant.run(ctrl, seconds, stop=lambda t, d: abs(d.qpos[0]) > 0.395)
    th = np.array([wrap(a) for a in log["th"]])
    tail = log["t"] >= seconds - hold
    finished = log["t"][-1] >= seconds - 2 * plant.dt
    ok = bool(finished and np.all(log["mode"][tail] == BALANCE) and np.all(np.abs(th[tail]) < math.radians(10)))
    caught = log["t"][np.argmax(log["mode"] == BALANCE)] if np.any(log["mode"] == BALANCE) else math.nan
    return {"ok": ok, "t_catch": float(caught), "max_x": float(np.max(np.abs(log["x"]))), "log": log,
            "peak_supply_A": float(np.max(log["i_supply"])), "peak_motor_A": float(np.max(np.abs(log["i_motor"])))}


def batch(p: dict, n: int = 10, **kw) -> list[dict]:
    return [swingup_trial(p, seed=s, **kw) for s in range(n)]


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p = model.load_params()
    results = batch(p)
    for i, r in enumerate(results):
        print(f"seed {i}: {'PASS' if r['ok'] else 'FAIL'}  caught at {r['t_catch']:.2f} s, max |x| {r['max_x'] * 1000:.0f} mm, "
              f"peak motor {r['peak_motor_A']:.1f} A, peak supply {r['peak_supply_A']:.1f} A")
    print(f"{sum(r['ok'] for r in results)}/10 succeeded (PLAN 1.5 needs >= 9)")

    log = results[0]["log"]
    sel = log["t"] < 5
    fig, ax = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    ax[0].plot(log["t"][sel], np.degrees([wrap(a) for a in log["th"][sel]])); ax[0].set_ylabel("θ (deg, 0 = up)")
    ax[1].plot(log["t"][sel], log["x"][sel] * 1000); ax[1].set_ylabel("cart x (mm)")
    ax[2].plot(log["t"][sel], log["duty"][sel]); ax[2].set_ylabel("duty")
    ax[3].plot(log["t"][sel], log["i_supply"][sel], label="supply"); ax[3].plot(log["t"][sel], log["i_motor"][sel], label="motor", alpha=0.6)
    ax[3].axhline(5.25, color="r", ls="--", lw=0.8, label="PSU trip starts (5.25 A)"); ax[3].set_ylabel("current (A)")
    ax[3].legend(fontsize=7); ax[3].set_xlabel("t (s)")
    for a in ax:
        a.axvspan(results[0]["t_catch"], 5, color="g", alpha=0.07)
    fig.suptitle("PLAN 1.5: energy swing-up → LQR (green = LQR), 20T, 30 cm, 500 Hz, 2 ms delay")
    fig.tight_layout()
    fig.savefig(model.SIM_DIR / "plots" / "1.5-swingup.png", dpi=90)
