"""PLAN 1.4 checks: linearisation agrees with the hand derivation; LQR balances from 5° for 30 s."""

import numpy as np
import pytest

import lqr
import model


def test_mujoco_linearisation_matches_hand_derivation():
    p = model.load_params()
    A1, B1 = lqr.linearise_mujoco(p)
    A2, B2 = lqr.linearise_by_hand(p)
    assert np.abs(A1 - A2).max() / np.abs(A2).max() < 5e-3
    assert np.abs(B1 - B2).max() / np.abs(B2).max() < 5e-3


@pytest.mark.parametrize("compliant", [False, True])
def test_lqr_balances_30s_from_5deg(compliant):
    r = lqr.balance_check(model.load_params(), rate_hz=500, compliant=compliant)
    assert r["ok"], (r["log"]["t"][-1], r["max_x"])
