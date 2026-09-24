"""Double swing-up: the saved 30T 30/20 trajectory, tracked by TVLQR, must hand over and balance,
including with the supply capped at 5 A (SPEC §11 Q6)."""

import model
import double_control
import double_trajopt


def test_double_swingup_tracks_and_balances_under_5A_cap():
    p = model.with_params(double_trajopt.layout_params(30, 0.30, 0.20), supply__current_cap=5.0)
    r = double_control.trial(p, double_trajopt.load("30T_30-20"))
    assert r["ok"], (r["max_x"], r["peak_supply_A"])
    assert r["max_x"] < 0.395
