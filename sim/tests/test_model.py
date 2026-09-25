"""PLAN 1.2 check: the pole falls (and swings) the way hand-calculated physics says it should."""

import math

import mujoco
import numpy as np

import model


def _sim(p, theta0, seconds):
    m = model.build_single(p)
    d = mujoco.MjData(m)
    d.qpos[1] = theta0
    hinge = []
    while d.time < seconds:
        mujoco.mj_step(m, d)  # duty stays 0: the motor just brakes by back-EMF, as a real idle H-bridge would
        hinge.append((d.time, d.qpos[1]))
    return np.array(hinge)


def _params_pinned_cart():
    # Make the cart effectively immovable so the pole matches the simple fixed-pivot formula.
    return model.with_params(model.load_params(), cart__mass=1e4, pole__pivot_damping=0.0)


def test_fall_from_upright_matches_cosh_growth():
    p = _params_pinned_cart()
    q = p["pole"]
    lam = math.sqrt(q["mass"] * 9.81 * q["length"] / 2 / model.pole_inertia_about_pivot(p))
    theta0 = math.radians(2)
    traj = _sim(p, theta0, 0.25)
    t, th = traj[-1]
    predicted = theta0 * math.cosh(lam * t)  # small-angle prediction, good while theta stays small
    assert abs(th - predicted) / predicted < 0.03, (th, predicted)


def test_hanging_period_matches_pendulum_formula():
    p = _params_pinned_cart()
    q = p["pole"]
    period = 2 * math.pi * math.sqrt(model.pole_inertia_about_pivot(p) / (q["mass"] * 9.81 * q["length"] / 2))
    traj = _sim(p, math.pi - math.radians(3), 5.0)
    swing = traj[:, 1] - math.pi
    ups = [traj[i, 0] for i in range(1, len(swing)) if swing[i - 1] < 0 <= swing[i]]  # upward zero crossings
    measured = np.mean(np.diff(ups))
    assert abs(measured - period) / period < 0.01, (measured, period)


def test_pole_falls_over_on_real_cart():
    # With the real cart mass the pole must still fall from 5 degrees, and pass horizontal within ~1 s.
    traj = _sim(model.load_params(), math.radians(5), 1.0)
    assert np.max(np.abs(traj[:, 1])) > math.pi / 2
