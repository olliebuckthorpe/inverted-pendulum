"""System ID (sim/sysid.py): the excitation matches the Pico's, and the fitting recovers a known rig."""
import math
import shutil
import subprocess

import numpy as np
import pytest

import model
import sysid

FW = model.SIM_DIR.parent / "firmware"
CASES = [(k, a, f0, f1, T, t) for (k, a, f0, f1, T) in sysid.TESTS.values() for t in np.linspace(0, T + 0.5, 157)]


@pytest.mark.skipif(not shutil.which("clang++"), reason="needs clang++")
def test_excitation_matches_the_firmware(tmp_path):
    src = tmp_path / "sid.cpp"
    src.write_text(f'#include "{FW}/core/control.h"\n#include <cstdio>\nint main(){{int k; float a,f0,f1,T,t;\n'
                   'while (scanf("%d %f %f %f %f %f",&k,&a,&f0,&f1,&T,&t)==6) printf("%.9g\\n", ctl::sysid_duty(k,a,f0,f1,T,t));}\n')
    exe = tmp_path / "sid"
    subprocess.run(["clang++", "-std=c++17", "-O2", str(src), "-o", str(exe)], check=True)
    out = subprocess.run([str(exe)], input="\n".join(" ".join(repr(float(v)) if i else str(v) for i, v in enumerate(c))
                                                     for c in CASES), capture_output=True, text=True, check=True).stdout
    cpp = np.array([float(v) for v in out.split()])
    py = np.array([sysid.excitation(*c) for c in CASES])
    assert np.max(np.abs(cpp - py)) < 1e-5


def test_steps_centre_on_the_start():
    k, a, f0, f1, T = sysid.TESTS["steps50"]
    u = np.array([sysid.excitation(k, a, f0, f1, T, t) for t in np.arange(0, 2 / f0, 0.002)])
    assert abs(u.mean()) < 0.01 * a                  # net drive over a whole cycle ~0: the cart returns


def test_period_ignores_sensor_noise():
    t = np.arange(0, 10, 0.002)
    th = math.pi + 0.3 * np.sin(2 * math.pi * t / 0.9) + np.radians(0.068) * np.random.default_rng(0).standard_normal(t.size)
    assert abs(sysid.period(t, th) - 0.9) < 0.005


def test_swing_fit_recovers_length_and_damping():
    p0 = model.load_params()
    truth = sysid.true_rig(p0)
    swings = [sysid.synth_swing(truth, 30), sysid.synth_swing(truth, 90, seed=1)]
    p, fit = sysid.fit_swing(p0, swings)
    assert abs(fit["pole.length"] / truth["pole"]["length"] - 1) < 0.01
    assert abs(fit["pole.pivot_damping"] / truth["pole"]["pivot_damping"] - 1) < 0.15
    val = sysid.validate(p, {}, swings)
    assert all(v["pass"] for v in val.values())
    # and the unfitted model must FAIL the same check, or the check proves nothing
    assert not all(v["pass"] for v in sysid.validate(p0, {}, swings).values())


def test_cart_fit_recovers_the_ratios_on_a_short_recording():
    """One 8 s step test (minutes for all five): the fit must bring the motion within the PLAN 4.3 limit where
    the unfitted params.yaml fails it, and recover the per-kg ratios the recordings actually determine."""
    p0 = model.load_params()
    truth = sysid.true_rig(p0)
    rec = {"steps50": sysid.synth_cart(truth, "steps50")}
    before = sysid.validate(p0, rec, [])["steps50"]["max_err_mm"]
    p, fit = sysid.fit_cart(p0, list(rec.values()), delays=(0.002, 0.003, 0.004))
    after = sysid.validate(p, rec, [])["steps50"]["max_err_mm"]
    assert after < 5.0 < before
    for k, v in sysid.ratios(truth).items():
        assert abs(fit["ratios"][k] / v - 1) < 0.10, k


def test_swing_model_matches_textbook_physics():
    """Anchored to physics, not to itself (the synthetic data uses the same model): a small swing about hanging
    has period 2π·sqrt(J / (m·g·L/2))."""
    p = model.load_params()
    q = p["pole"]
    t = np.arange(0, 10, 0.002)
    th = sysid.swing_model(model.with_params(p, pole__pivot_damping=0.0), t, math.pi + math.radians(3), 0.0)
    expected = 2 * math.pi * math.sqrt(model.pole_inertia_about_pivot(p) / (q["mass"] * 9.81 * q["length"] / 2))
    assert abs(np.degrees(th - math.pi)).max() < 3.1          # it swings about hanging, not away from it
    assert abs(sysid.period(t, th, h=math.radians(0.5)) / expected - 1) < 0.005
