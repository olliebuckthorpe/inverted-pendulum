"""The sympy equations of motion for the double must match MuJoCo's accelerations at random states."""

import mujoco
import numpy as np

import double_eom
import model


def test_sympy_double_matches_mujoco():
    p = model.load_params()
    m = model.build_double(p)
    m.dof_frictionloss[:] = 0.0  # Coulomb friction isn't in the smooth equations
    d = mujoco.MjData(m)
    f_np = double_eom.numpy_dynamics(p)
    f_ca = double_eom.casadi_dynamics(p)
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform([-0.3, -np.pi, -np.pi], [0.3, np.pi, np.pi])
        qd = rng.uniform(-8, 8, 3) * [0.2, 1, 1]
        u = rng.uniform(-1, 1)
        d.qpos[:], d.qvel[:], d.ctrl[0] = q, qd, u
        mujoco.mj_forward(m, d)
        ours = f_np(q, qd, u)
        ours_ca = np.array(f_ca(np.concatenate([q, qd]), u)).ravel()[3:]
        scale = np.abs(d.qacc).max() + 1.0
        assert np.abs(ours - d.qacc).max() / scale < 1e-6, (q, qd, u, ours, d.qacc)
        assert np.abs(ours_ca - d.qacc).max() / scale < 1e-6
