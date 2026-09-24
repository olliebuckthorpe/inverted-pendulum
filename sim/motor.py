"""DC motor model from the Pololu 37D datasheet (PLAN 1.3).

The force itself is computed inside MuJoCo (the actuator in model.py), so the physics and the LQR
linearisation both see it. This file adds the electrical side that MuJoCo doesn't know about:

    voltage   V = duty × supply voltage
    current   I = (V − Ke·ω) / R          ω = gearbox output speed = cart speed / pulley radius
    torque    τ = Kt · I
    force     F = τ / r

and from those, the supply current and the optional PSU current cap (SPEC §11 Q6).

The current is NOT clamped at the 5.5 A stall figure. Reversing at speed puts the supply voltage and
the back-EMF in the same direction, so the current really can reach ~(12 + 12)/2.18 ≈ 11 A.
"""

from __future__ import annotations

import math

import model


def motor_current(p: dict, duty: float, cart_speed: float) -> float:
    """Winding current (A) for a duty cycle in [-1, 1] and the drive's speed (m/s)."""
    m = p["motor"]
    omega = cart_speed / model.pulley_radius(p)
    return (duty * p["supply"]["voltage"] - m["Ke"] * omega) / m["R"]


def supply_current(p: dict, duty: float, cart_speed: float) -> float:
    """Current drawn from the PSU (A). Negative means the motor is pushing energy back (regeneration).

    An H-bridge switching at duty d connects the motor to the supply for a fraction |d| of the time,
    so on average the supply sees |d| × the motor current, in the direction the bridge is driving.
    (Ignores switching losses and the driver's own few mA.)
    """
    return duty * motor_current(p, duty, cart_speed)


def cap_duty_for_supply(p: dict, duty: float, cart_speed: float, cap: float | None) -> float:
    """Shrink |duty| until the supply current is at most `cap` amps. None = no cap.

    Supply current i(d) = d·(V·d − Ke·ω)/R. Setting i(d) = cap gives the quadratic
        V·d² − Ke·ω·d − R·cap = 0
    whose two roots bound the duty cycles that keep the supply current under the cap.
    """
    if cap is None:
        return duty
    m = p["motor"]
    V, R = p["supply"]["voltage"], m["R"]
    b = m["Ke"] * cart_speed / model.pulley_radius(p)  # the back-EMF, in volts
    disc = math.sqrt(b * b + 4 * V * R * cap)
    lo, hi = (b - disc) / (2 * V), (b + disc) / (2 * V)
    return min(max(duty, lo), hi)
