"""Equations of motion for the double pendulum on the cart, derived with sympy (Lagrange's method).

Coordinates q = [x, a1, a2]: cart position, rod 1 angle from upright, rod 2 angle RELATIVE to rod 1
(what the joint-2 sensor measures). Upright = [x, 0, 0]; both hanging = [x, pi, 0].

Method: write kinetic energy T and potential energy V, form L = T − V, then for each coordinate
    d/dt(∂L/∂q_dot) − ∂L/∂q = Q        (Q = motor + friction forces)
which rearranges to  M(q)·q_dd = Q − h(q, q_dot).  M is the mass matrix, h the gravity/centrifugal terms.

The result is used by the trajectory optimiser (CasADi) and cross-checked against MuJoCo in tests.
"""

from __future__ import annotations

from functools import lru_cache

import casadi as ca
import numpy as np
import sympy as sp

import model

G_ACC = 9.81


@lru_cache(maxsize=1)
def _symbolic():
    t = sp.symbols("t")
    x, a1, a2 = (sp.Function(n)(t) for n in ("x", "a1", "a2"))
    q = [x, a1, a2]
    qd = [sp.diff(v, t) for v in q]
    # physical parameters as symbols, so one derivation serves every params.yaml
    M, m1, L1, I1, mj, Ij, m2, L2, I2, Ih = sp.symbols("M m1 L1 I1 mj Ij m2 L2 I2 Ih")
    Gm, Dm, c, b1, b2, u = sp.symbols("Gm Dm c b1 b2 u")

    # Positions: rod 1 centre, joint 2, rod 2 centre (x to the right, z up, angles tip toward +x)
    p1 = (x + L1 / 2 * sp.sin(a1), L1 / 2 * sp.cos(a1))
    pj = (x + L1 * sp.sin(a1), L1 * sp.cos(a1))
    p2 = (pj[0] + L2 / 2 * sp.sin(a1 + a2), pj[1] + L2 / 2 * sp.cos(a1 + a2))
    v2 = lambda p: sp.diff(p[0], t) ** 2 + sp.diff(p[1], t) ** 2

    T = (sp.Rational(1, 2) * M * qd[0] ** 2                       # cart + hub + reflected rotor
         + sp.Rational(1, 2) * Ih * qd[1] ** 2                    # hub sphere spinning with rod 1
         + sp.Rational(1, 2) * (m1 * v2(p1) + I1 * qd[1] ** 2)    # rod 1
         + sp.Rational(1, 2) * (mj * v2(pj) + Ij * (qd[1] + qd[2]) ** 2)   # joint-2 hardware (turns with rod 2)
         + sp.Rational(1, 2) * (m2 * v2(p2) + I2 * (qd[1] + qd[2]) ** 2))  # rod 2
    V = G_ACC * (m1 * p1[1] + mj * pj[1] + m2 * p2[1])
    Lag = T - V
    Q = [Gm * u - (Dm + c) * qd[0], -b1 * qd[1], -b2 * qd[2]]

    # Replace functions of t with plain symbols so the result can be compiled
    qs = sp.symbols("x a1 a2"); qds = sp.symbols("xd a1d a2d"); qdds = sp.symbols("xdd a1dd a2dd")
    subs = {}
    for f, s, sd, sdd in zip(q, qs, qds, qdds):
        subs[sp.diff(f, t, 2)] = sdd
        subs[sp.diff(f, t)] = sd
        subs[f] = s
    eqs = [sp.expand(sp.diff(sp.diff(Lag, qdi), t) - sp.diff(Lag, qi) - Qi).subs(subs)
           for qi, qdi, Qi in zip(q, qd, Q)]
    Mmat = sp.Matrix([[sp.simplify(sp.diff(e, a)) for a in qdds] for e in eqs])
    rest = sp.Matrix([sp.simplify(e.subs({a: 0 for a in qdds})) for e in eqs])  # M·q_dd + rest = 0
    params = (M, m1, L1, I1, mj, Ij, m2, L2, I2, Ih, Gm, Dm, c, b1, b2)
    return qs, qds, u, params, Mmat, rest


def param_values(p: dict) -> tuple:
    c, q, j2, q2 = p["cart"], p["pole"], p["joint2"], p["pole2"]
    rod_I = lambda m, L, r: m * (3 * r * r + L * L) / 12  # solid cylinder about its centre
    return (c["mass"] + q["hub_mass"] + model.reflected_rotor_mass(p),
            q["mass"], q["length"], rod_I(q["mass"], q["length"], q["radius"]),
            j2["mass"], 0.4 * j2["mass"] * 0.01 ** 2,
            q2["mass"], q2["length"], rod_I(q2["mass"], q2["length"], q2["radius"]),
            0.4 * q["hub_mass"] * 0.012 ** 2,
            model.motor_gain(p), model.motor_damping(p), c["viscous_friction"],
            q["pivot_damping"], j2["damping"])


def numpy_dynamics(p: dict):
    """Return f(q, qd, u) -> q_dd as numpy (Coulomb friction not included)."""
    qs, qds, u, params, Mmat, rest = _symbolic()
    vals = dict(zip(params, param_values(p)))
    fM = sp.lambdify((qs, qds, u), Mmat.subs(vals), "numpy")
    fr = sp.lambdify((qs, qds, u), rest.subs(vals), "numpy")

    def f(qv, qdv, uv):
        return np.linalg.solve(np.array(fM(qv, qdv, uv), float), -np.array(fr(qv, qdv, uv), float).ravel())
    return f


def casadi_dynamics(p: dict) -> ca.Function:
    """CasADi function f(state[6], u) -> state_dot[6], state = [x, a1, a2, xd, a1d, a2d]."""
    qs, qds, u, params, Mmat, rest = _symbolic()
    vals = dict(zip(params, param_values(p)))
    mods = [{"sin": ca.sin, "cos": ca.cos}, "math"]
    fM = sp.lambdify((qs, qds, u), Mmat.subs(vals).tolist(), modules=mods)
    fr = sp.lambdify((qs, qds, u), rest.subs(vals).tolist(), modules=mods)
    s = ca.SX.sym("s", 6); uu = ca.SX.sym("u")
    qv, qdv = [s[i] for i in range(3)], [s[i] for i in range(3, 6)]
    Mx = ca.SX(3, 3); rx = ca.SX(3, 1)
    Mn, rn = fM(qv, qdv, uu), fr(qv, qdv, uu)
    for i in range(3):
        rx[i] = rn[i][0]
        for j in range(3):
            Mx[i, j] = Mn[i][j]
    qdd = ca.solve(Mx, -rx)
    return ca.Function("f", [s, uu], [ca.vertcat(s[3:], qdd)])
