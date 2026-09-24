"""Classical double swing-up: track the optimised trajectory with time-varying LQR, then hand over to the
double's balance LQR (Tedrake, Underactuated Robotics ch. 8 + 10).

State (same order as double_trajopt): s = [x, a1, a2, x_dot, a1_dot, a2_dot], a2 relative to rod 1.

TVLQR in plain terms: the plan says where the rig should be at every instant. Near that plan the rig
behaves linearly, s_err(k+1) ≈ A_k·s_err(k) + B_k·u_err(k), with A_k, B_k changing along the path.
Running the LQR (Riccati) equation BACKWARDS from the end of the swing-up gives a gain K_k for every
control step; the command is  u = u_plan(t) − K_k·(s − s_plan(t)).  At the end the gains match the
ordinary balance LQR, so the hand-over is smooth.

Run:  uv run python double_control.py [trajectory name, e.g. 30T_30-20]
"""

from __future__ import annotations

import math
from collections import deque

import casadi as ca
import control
import mujoco
import numpy as np

import double_eom
import double_trajopt
import model
import motor
from plant import wrap

Q_BAL = np.diag([10.0, 200.0, 200.0, 1.0, 5.0, 5.0])   # balance weights over [x, a1, a2, xd, a1d, a2d]
R_BAL = np.array([[1.0]])
Q_TRK = np.diag([20.0, 50.0, 50.0, 1.0, 2.0, 2.0])     # tracking weights (looser on angles mid-swing)
R_TRK = np.array([[1.0]])


def jacobians(p: dict):
    f = double_eom.casadi_dynamics(p)
    s, u = ca.SX.sym("s", 6), ca.SX.sym("u")
    fx = f(s, u)
    return ca.Function("AB", [s, u], [ca.jacobian(fx, s), ca.jacobian(fx, u)])


def balance_gain(p: dict, rate_hz: float) -> np.ndarray:
    AB = jacobians(p)
    A, B = (np.array(M) for M in AB(np.zeros(6), 0.0))
    sysd = control.c2d(control.ss(A, B, np.eye(6), 0), 1.0 / rate_hz, method="zoh")
    K, S, _ = control.dlqr(sysd.A, sysd.B, Q_BAL, R_BAL)
    return np.asarray(K), np.asarray(S)


def tvlqr(p: dict, traj: dict, rate_hz: float) -> dict:
    """Time-varying gains along the trajectory, sampled at the controller rate."""
    AB = jacobians(p)
    dt = 1.0 / rate_hz
    T = float(traj["T"])
    ts = np.arange(0.0, T, dt)
    X = np.array([np.interp(ts, traj["t"], traj["X"][i]) for i in range(6)]).T
    U = np.interp(ts, traj["t"], traj["U"])
    K_end, S = balance_gain(p, rate_hz)
    Ks = np.zeros((len(ts), 6))
    for k in range(len(ts) - 1, -1, -1):
        A, B = (np.array(M) for M in AB(X[k], U[k]))
        Ad = np.eye(6) + A * dt + A @ A * dt * dt / 2   # one-step discretisation (dt = 1-2 ms)
        Bd = B * dt + A @ B * dt * dt / 2
        K = np.linalg.solve(R_TRK + Bd.T @ S @ Bd, Bd.T @ S @ Ad)
        S = Q_TRK + Ad.T @ S @ (Ad - Bd @ K)
        S = (S + S.T) / 2
        Ks[k] = K.ravel()
    return {"t": ts, "X": X, "U": U, "K": Ks, "K_bal": K_end.ravel(), "T": T}


class SwingUpThenBalance:
    """mode 2 = following the swing-up plan, mode 1 = balance LQR (after the hand-over)."""

    def __init__(self, p: dict, gains: dict, end_angles=(0.0, 0.0)):
        self.g = gains
        self.end = np.array([0.0, *end_angles, 0.0, 0.0, 0.0])
        self.mode = 2
        self.F_coulomb = (p["cart"]["coulomb_friction"] + model.gearbox_drag(p)) / model.motor_gain(p)

    def __call__(self, t: float, s: np.ndarray) -> float:
        g = self.g
        if t < g["T"]:
            k = min(int(t * len(g["t"]) / g["T"]), len(g["t"]) - 1)
            err = s - g["X"][k]
            err[1], err[2] = wrap(err[1]), wrap(err[2])
            u = g["U"][k] - g["K"][k] @ err
            # friction feed-forward: the plan assumed no dry friction, so add the duty it costs
            u += self.F_coulomb * np.sign(g["X"][k][3]) if abs(g["X"][k][3]) > 0.01 else 0.0
            return float(u)
        self.mode = 1
        err = s - self.end
        err[1], err[2] = wrap(err[1]), wrap(err[2])
        return float(-g["K_bal"] @ err)


class DoublePlant:
    """The double rig as the Pico sees it: fixed rate, delay, quantised sensors, filtered velocities."""

    def __init__(self, p: dict, rate_hz: float = 1000.0, seed: int = 0, delay: float | None = None):
        self.p = p
        self.m = model.build_double(p)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.n = max(1, round(1.0 / (rate_hz * self.dt)))
        self.ctrl_dt = self.n * self.dt
        delay = p["sim"]["control_delay"] if delay is None else delay
        self.delay_steps = round(delay / self.dt)
        s = p["sensing"]
        self.steps = np.array([2 * math.pi * model.pulley_radius(p) / s["cart_counts_per_rev"],
                               2 * math.pi / 2 ** s["angle_bits"], 2 * math.pi / 2 ** s["angle_bits"]])
        self.alpha = self.ctrl_dt / (self.ctrl_dt + 1 / (2 * math.pi * s["velocity_filter_hz"]))
        self.cap = p["supply"]["current_cap"]
        self.rng = np.random.default_rng(seed)

    def reset(self, q=(0.0, math.pi, 0.0), qd=(0.0, 0.0, 0.0)):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:], self.d.qvel[:] = q, qd
        mujoco.mj_forward(self.m, self.d)
        self.prev = np.round(np.array(q) / self.steps) * self.steps
        self.pos = self.prev.copy()   # continuous (unwrapped) measured positions
        self.vel = np.zeros(3)
        self.queue = deque([0.0] * self.delay_steps)

    def measure(self) -> np.ndarray:
        raw = np.array(self.d.qpos[:3])
        # the angle sensors only report 0..2π: wrap them, then unwrap by accumulating the change
        meas = np.round(raw / self.steps) * self.steps
        delta = meas - self.prev
        delta[1:] = [wrap(v) for v in delta[1:]]
        self.prev = meas
        self.pos = self.pos + delta
        self.vel += self.alpha * (delta / self.ctrl_dt - self.vel)
        return np.concatenate([self.pos, self.vel])

    def run(self, controller, seconds: float, stop=None) -> dict:
        log = {k: [] for k in ("t", "q", "qd", "duty", "i_supply", "mode")}
        duty_cmd = 0.0
        for k in range(round(seconds / self.dt)):
            if k % self.n == 0:
                duty_cmd = float(np.clip(controller(self.d.time, self.measure()), -1.0, 1.0))
            self.queue.append(duty_cmd)
            duty = motor.cap_duty_for_supply(self.p, self.queue.popleft(), self.d.qvel[0], self.cap)
            self.d.ctrl[0] = duty
            log["t"].append(self.d.time); log["q"].append(self.d.qpos[:3].copy()); log["qd"].append(self.d.qvel[:3].copy())
            log["duty"].append(duty); log["i_supply"].append(motor.supply_current(self.p, duty, self.d.qvel[0]))
            log["mode"].append(getattr(controller, "mode", 0))
            mujoco.mj_step(self.m, self.d)
            if stop is not None and stop(self.d):
                break
        return {k: np.asarray(v) for k, v in log.items()}


def trial(p: dict, traj: dict, rate_hz: float = 1000.0, seconds: float | None = None, gains=None,
          start=(0.0, math.pi, 0.0), seed: int = 0) -> dict:
    gains = gains or tvlqr(p, traj, rate_hz)
    end = (traj["X"][1, -1], traj["X"][2, -1])
    plant = DoublePlant(p, rate_hz=rate_hz, seed=seed)
    plant.reset(q=start)
    seconds = seconds or gains["T"] + 5.0
    log = plant.run(SwingUpThenBalance(p, gains, end), seconds, stop=lambda d: abs(d.qpos[0]) > 0.395)
    a1 = np.array([wrap(v) for v in log["q"][:, 1]])
    a2 = np.array([wrap(v) for v in log["q"][:, 2]])
    tail = log["t"] >= log["t"][-1] - 2.0
    finished = log["t"][-1] >= seconds - 2 * plant.dt
    ok = bool(finished and np.all(np.abs(a1[tail]) < math.radians(10)) and np.all(np.abs(a2[tail]) < math.radians(10)))
    return {"ok": ok, "log": log, "max_x": float(np.abs(log["q"][:, 0]).max()),
            "peak_supply_A": float(log["i_supply"].max()), "gains": gains}


if __name__ == "__main__":
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "30T_30-20"
    teeth, rods = name.split("_")
    bottom, top = (int(v) / 100 for v in rods.split("-"))
    p = double_trajopt.layout_params(int(teeth[:-1]), bottom, top)
    traj = double_trajopt.load(name)
    r = trial(p, traj)
    print(f"{name}: {'BALANCED' if r['ok'] else 'FAILED'} after a {traj['T']:.2f} s swing-up; "
          f"max|x| {r['max_x'] * 1000:.0f} mm, peak supply {r['peak_supply_A']:.1f} A")
