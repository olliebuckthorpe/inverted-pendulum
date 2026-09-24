"""PLAN 1.3 unit check: the datasheet motor model gives the right stall force and top speed."""

import math

import mujoco
import pytest

import model
import motor
from plant import Plant


@pytest.mark.parametrize("teeth, force", [(20, 46.0), (30, 31.0)])
def test_locked_cart_full_duty_force(teeth, force):
    p = model.load_params(**{"drive.pulley_teeth": teeth})
    m = model.build_single(p)
    d = mujoco.MjData(m)
    d.ctrl[0] = 1.0
    mujoco.mj_forward(m, d)  # cart at rest = locked, so no back-EMF yet
    assert d.actuator_force[0] == pytest.approx(force, rel=0.01)
    assert motor.motor_current(p, 1.0, 0.0) == pytest.approx(5.5, rel=0.01)  # the datasheet stall current


@pytest.mark.parametrize("teeth, speed", [(20, 1.07), (30, 1.60)])
def test_free_cart_top_speed(teeth, speed):
    # Long rail, no rail friction: only the gearbox drag from the no-load current remains, as on the datasheet.
    p = model.load_params(**{"drive.pulley_teeth": teeth, "cart.travel": 100.0,
                             "cart.coulomb_friction": 0.0, "cart.viscous_friction": 0.0})
    plant = Plant(p, rate_hz=1000, delay=0.0)
    plant.reset(th=math.pi)  # pole hanging
    log = plant.run(lambda t, obs: 1.0, seconds=1.5)
    assert log["xd"][-1] == pytest.approx(speed, rel=0.01)
    # At top speed the motor draws only the no-load current.
    assert log["i_motor"][-1] == pytest.approx(0.2, abs=0.03)


def test_reversal_at_speed_exceeds_stall_current():
    p = model.load_params()
    v = 1.0  # m/s forwards, then full reverse duty
    assert motor.motor_current(p, -1.0, v) < -9.5


def test_supply_cap_limits_current():
    p = model.load_params()
    for v in (-1.0, 0.0, 0.5, 1.0):
        for duty in (-1.0, -0.5, 0.5, 1.0):
            capped = motor.cap_duty_for_supply(p, duty, v, 5.0)
            assert motor.supply_current(p, capped, v) <= 5.0 + 1e-9
            assert abs(capped) <= abs(duty) + 1e-12
