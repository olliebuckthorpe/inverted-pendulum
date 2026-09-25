"""LQR balance for the single pendulum (PLAN 1.4).

State s = [x, x_dot, theta, theta_dot] (PLAN order; theta = 0 upright). Input u = duty cycle in [-1, 1].

Near upright the rig behaves like  s_dot = A s + B u.  LQR picks the gain K for u = -K s that minimises
the integral of  s'Qs + u'Ru : Q = how much each error hurts, R = how much motor use hurts.

Run:  uv run python lqr.py        (balances from 5°, prints the check from PLAN 1.4 and saves a plot)
"""

from __future__ import annotations

import math

import control
import mujoco
import numpy as np

import model
from plant import Plant

# MuJoCo orders the state as [x, theta, x_dot, theta_dot]; the plan (and the Pico) use [x, x_dot, theta, theta_dot].
MJ_TO_PLAN = [0, 2, 1, 3]

Q_DEFAULT = np.diag([10.0, 1.0, 100.0, 1.0])   # PLAN 1.4 starting weights over [x, x_dot, theta, theta_dot]
R_DEFAULT = np.array([[1.0]])


def linearise_mujoco(p: dict) -> tuple[np.ndarray, np.ndarray]:
    """Continuous-time A, B from MuJoCo by finite differences about upright (motor included).

    Coulomb friction is switched off first: it's a sudden +/- step at zero speed, which has no
    straight-line approximation, and LQR ignores it anyway (the controller just has to push through it).
    """
    m = model.build_single(p)
    m.dof_frictionloss[:] = 0.0
    d = mujoco.MjData(m)

    def accel(qpos, qvel, ctrl):
        d.qpos[:], d.qvel[:], d.ctrl[:] = qpos, qvel, ctrl
        mujoco.mj_forward(m, d)
        return d.qacc.copy()

    # Nudge each state and the input either side of upright-at-rest and see how the accelerations change
    # (central differences). Positions change at the rate of the velocities, so those rows are exact.
    eps = 1e-6
    z = np.zeros(m.nv)
    Ac = np.zeros((4, 4)); Bc = np.zeros((4, 1))
    Ac[0, 2] = Ac[1, 3] = 1.0
    for i in range(m.nv):
        e = np.zeros(m.nv); e[i] = eps
        Ac[2:, i] = (accel(e, z, [0]) - accel(-e, z, [0])) / (2 * eps)
        Ac[2:, 2 + i] = (accel(z, e, [0]) - accel(z, -e, [0])) / (2 * eps)
    Bc[2:, 0] = (accel(z, z, [eps]) - accel(z, z, [-eps])) / (2 * eps)
    return Ac[np.ix_(MJ_TO_PLAN, MJ_TO_PLAN)], Bc[MJ_TO_PLAN]


def linearise_by_hand(p: dict) -> tuple[np.ndarray, np.ndarray]:
    """The same A, B from the equations of motion (a cross-check on the MuJoCo numbers).

    With M = cart + hub + reflected rotor mass, pole mass m, pivot-to-centre-of-mass l, pole inertia
    about the pivot J, motor force F = G·u − D·x_dot, viscous friction c (rail) and b (pivot),
    linearised about upright (sin θ ≈ θ, cos θ ≈ 1, θ_dot² ≈ 0):

        (M + m)·x_dd + m·l·θ_dd = G·u − (D + c)·x_dot
         m·l·x_dd    +   J·θ_dd = m·g·l·θ − b·θ_dot

    Solve the 2×2 system for x_dd and θ_dd, then read off A and B.
    """
    q, c = p["pole"], p["cart"]
    M = c["mass"] + q["hub_mass"] + model.reflected_rotor_mass(p)
    m, l, g = q["mass"], q["length"] / 2, 9.81
    J = model.pole_inertia_about_pivot(p)
    G, D = model.motor_gain(p), model.motor_damping(p)
    cv, b = c["viscous_friction"], q["pivot_damping"]
    Minv = np.linalg.inv(np.array([[M + m, m * l], [m * l, J]]))
    # Right-hand sides as linear functions of [x, x_dot, theta, theta_dot] and u
    rhs_s = np.array([[0, -(D + cv), 0, 0], [0, 0, m * g * l, -b]])
    rhs_u = np.array([[G], [0]])
    acc_s, acc_u = Minv @ rhs_s, Minv @ rhs_u
    A = np.array([[0, 1, 0, 0], acc_s[0], [0, 0, 0, 1], acc_s[1]], dtype=float)
    B = np.array([[0], acc_u[0], [0], acc_u[1]], dtype=float)
    return A, B


def discretise(A: np.ndarray, B: np.ndarray, period: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact zero-order-hold discretisation: what one control period does, with u held constant."""
    sys = control.c2d(control.ss(A, B, np.eye(4), 0), period, method="zoh")
    return np.asarray(sys.A), np.asarray(sys.B)


def design(p: dict, rate_hz: float, Q=Q_DEFAULT, R=R_DEFAULT) -> np.ndarray:
    """Discrete LQR gain K (1×4, PLAN state order) for a controller running at rate_hz."""
    A, B = linearise_mujoco(p)
    Ad, Bd = discretise(A, B, 1.0 / rate_hz)
    K, _, _ = control.dlqr(Ad, Bd, Q, R)
    return np.asarray(K)


class LQRController:
    mode = 1

    def __init__(self, K: np.ndarray, x_ref: float = 0.0):
        self.K = K.ravel()
        self.x_ref = x_ref

    def __call__(self, t: float, obs: np.ndarray) -> float:
        s = obs - np.array([self.x_ref, 0, 0, 0])
        return float(-self.K @ s)


def fell(limit_deg: float = 45.0):
    """Stop condition: pole beyond limit_deg from upright, or cart at the rail end."""
    def stop(t, d):
        th = math.atan2(math.sin(d.qpos[1]), math.cos(d.qpos[1]))
        return abs(th) > math.radians(limit_deg) or abs(d.qpos[0]) > 0.395
    return stop


def balance_check(p: dict, rate_hz: float = 500.0, compliant: bool = False, seconds: float = 30.0,
                  start_deg: float = 5.0, K=None) -> dict:
    K = design(p, rate_hz) if K is None else K
    plant = Plant(p, rate_hz=rate_hz, compliant=compliant)
    plant.reset(th=math.radians(start_deg))
    log = plant.run(LQRController(K), seconds, stop=fell())
    ok = log["t"][-1] >= seconds - 2 * plant.dt and np.max(np.abs(log["x"])) <= 0.2
    return {"ok": bool(ok), "K": K, "log": log, "max_x": float(np.max(np.abs(log["x"])))}


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p = model.load_params()
    A1, B1 = linearise_mujoco(p)
    A2, B2 = linearise_by_hand(p)
    np.set_printoptions(precision=3, suppress=True)
    print("A (MuJoCo)\n", A1, "\nB (MuJoCo)\n", B1.ravel())
    rel = lambda X, Y: np.abs(X - Y).max() / np.abs(Y).max()
    print("relative difference vs hand derivation: A %.1e, B %.1e" % (rel(A1, A2), rel(B1, B2)))
    print("open-loop poles (rad/s):", np.linalg.eigvals(A1))

    for compliant in (False, True):
        r = balance_check(p, compliant=compliant)
        log = r["log"]
        print(f"{'compliant' if compliant else 'rigid    '}: K = {r['K'].ravel()}  "
              f"balanced {log['t'][-1]:.1f} s, max |x| = {r['max_x'] * 1000:.1f} mm, "
              f"peak |duty| {np.max(np.abs(log['duty'])):.2f} -> {'PASS' if r['ok'] else 'FAIL'}")

    r = balance_check(p)
    log = r["log"]
    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    sel = log["t"] < 3
    ax[0].plot(log["t"][sel], np.degrees(log["th"][sel])); ax[0].set_ylabel("θ (deg)")
    ax[1].plot(log["t"][sel], log["x"][sel] * 1000); ax[1].set_ylabel("cart x (mm)")
    ax[2].plot(log["t"][sel], log["duty"][sel]); ax[2].set_ylabel("duty"); ax[2].set_xlabel("t (s)")
    fig.suptitle("PLAN 1.4: LQR balance from 5°, 500 Hz, 2 ms delay, quantised sensors (first 3 s of 30)")
    fig.tight_layout()
    fig.savefig(model.SIM_DIR / "plots" / "1.4-lqr-balance.png", dpi=90)
