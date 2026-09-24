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


def test_kalman_filter_handles_datasheet_sensor_noise():
    # AS5047P datasheet noise (0.068° RMS): plain differencing fails, the Kalman filter must not.
    p = model.with_params(double_trajopt.layout_params(30, 0.30, 0.20), supply__current_cap=5.0,
                          sensing__angle_noise_deg=0.068)
    traj = double_trajopt.load("30T_30-20")
    g = double_control.tvlqr(p, traj, 500.0)
    kf = double_control.kalman_gains(p, g)
    assert all(double_control.trial(p, traj, rate_hz=500.0, gains=g, kf=kf, seed=s)["ok"] for s in range(3))
    assert not double_control.trial(p, traj, rate_hz=500.0, gains=g, seed=0)["ok"]  # the problem it solves


def test_chosen_plan_survives_random_post_sysid_rigs():
    # The chosen plan (gentle + joint-2 friction x1.5) with the Kalman filter, on random rigs with every
    # parameter wrong at once within post-system-ID errors. 50/50 at full size; check a quick 8 here.
    import double_montecarlo
    p = double_trajopt.layout_params(30, 0.30, 0.20)
    r = double_montecarlo.run("30T_30-20_damping150_u60_x30", n=8, scale=1.0,
                              design_overrides={"joint2__damping": p["joint2"]["damping"] * 1.5})
    assert r["success"] == 8
