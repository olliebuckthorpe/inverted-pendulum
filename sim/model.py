"""Build the MuJoCo cart-pole model from params.yaml (PLAN 1.2).

Two variants of the same rig:

- rigid (default): the cart has one sliding joint and the motor pushes it directly. The motor's
  spinning rotor is added to the cart as extra "reflected" mass (MuJoCo's `armature`). Used for
  LQR design and for most runs.
- compliant: the motor drives a separate "drive" slider (the belt tooth under the pulley) and a stiff
  spring with a dead-band (the gearbox backlash) connects it to the cart. That spring force is applied
  from Python in plant.py. Used to check the controllers survive backlash and belt stretch.

Angle convention: the hinge angle is 0 with the pole straight UP and increases as the top tips toward +x.
Hanging straight down is ±pi.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import mujoco
import yaml

SIM_DIR = Path(__file__).resolve().parent
PARAMS_PATH = SIM_DIR / "params.yaml"
MODELS_DIR = SIM_DIR / "models"


def load_params(path: Path = PARAMS_PATH, **overrides) -> dict:
    """Read params.yaml. `overrides` uses dotted keys, e.g. {"drive.pulley_teeth": 30}."""
    with open(path) as f:
        p = yaml.safe_load(f)
    for dotted, value in overrides.items():
        set_param(p, dotted, value)
    for section, values in p.items():
        for key, value in values.items():
            # PyYAML reads "3.0e4" as a string (YAML 1.1 wants "3.0e+4"), so catch that here.
            if value is not None and not isinstance(value, (int, float)):
                raise TypeError(f"params.yaml {section}.{key} = {value!r} is not a number")
    return p


def set_param(p: dict, dotted: str, value) -> None:
    section, key = dotted.split(".")
    if key not in p[section]:
        raise KeyError(f"unknown parameter {dotted}")
    p[section][key] = value


def with_params(p: dict, **overrides) -> dict:
    q = copy.deepcopy(p)
    for dotted, value in overrides.items():
        set_param(q, dotted.replace("__", "."), value)
    return q


# ---------------------------------------------------------------------------------------------------
# Derived quantities. Each is one line of physics, so they live here in one place.
# ---------------------------------------------------------------------------------------------------

def pulley_radius(p: dict) -> float:
    """GT2 pulley pitch radius: circumference = teeth × 2 mm, so r = teeth × pitch / 2π."""
    return p["drive"]["pulley_teeth"] * p["drive"]["belt_pitch"] / (2 * math.pi)


def motor_gain(p: dict) -> float:
    """Force on the cart per unit duty when the cart is still: F = Kt · (V/R) / r."""
    m = p["motor"]
    return m["Kt"] * p["supply"]["voltage"] / (m["R"] * pulley_radius(p))


def motor_damping(p: dict) -> float:
    """Back-EMF braking, as a force per cart speed.

    A spinning motor makes a voltage Ke·ω that opposes the supply, so the current (and force) falls
    as speed rises: F = Kt·(V − Ke·ω)/(R·r) with ω = v/r. The speed-dependent part is −D·v with
    D = Kt·Ke / (R·r²). This is why the cart has a top speed.
    """
    m = p["motor"]
    r = pulley_radius(p)
    return m["Kt"] * m["Ke"] / (m["R"] * r * r)


def reflected_rotor_mass(p: dict) -> float:
    """The motor rotor's inertia, as it feels from the cart: J · N² / r².

    Accelerating the cart at a spins the rotor up at a·N/r, which takes torque J·a·N/r at the rotor.
    That's N times bigger at the gearbox output, divided by r to become a force: J·N²/r² · a.
    """
    m = p["motor"]
    r = pulley_radius(p)
    return m["rotor_inertia"] * m["gear_ratio"] ** 2 / (r * r)


def gearbox_drag(p: dict) -> float:
    """Constant (Coulomb) drag force from gearbox friction, from the datasheet no-load current."""
    m = p["motor"]
    return m["Kt"] * m["I_noload"] / pulley_radius(p)


def backlash_halfwidth(p: dict) -> float:
    """Half the gearbox play, converted to cart travel (m)."""
    return 0.5 * math.radians(p["motor"]["backlash_deg"]) * pulley_radius(p)


def pole_inertia_about_pivot(p: dict) -> float:
    """Solid cylinder about its end: m(3r² + L²)/12 about the centre, plus m(L/2)² (parallel axis)."""
    q = p["pole"]
    m, L, r = q["mass"], q["length"], q["radius"]
    return m * (3 * r * r + L * L) / 12 + m * (L / 2) ** 2


# ---------------------------------------------------------------------------------------------------
# MJCF
# ---------------------------------------------------------------------------------------------------

def single_xml(p: dict, compliant: bool = False) -> str:
    c, q, s = p["cart"], p["pole"], p["sim"]
    travel = c["travel"]
    rail_friction = c["coulomb_friction"]
    rail_damping = c["viscous_friction"]
    G, D = motor_gain(p), motor_damping(p)

    pole_body = f"""
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" limited="false" damping="{q['pivot_damping']}" frictionloss="{q.get('pivot_coulomb', 0.0)}" stiffness="{q.get('cable_stiffness', 0.0)}" springref="{math.pi}"/>
        <geom name="hub" type="sphere" size="0.012" mass="{q['hub_mass']}" rgba="0.3 0.3 0.3 1"/>
        <geom name="pole" type="cylinder" fromto="0 0 0 0 0 {q['length']}" size="{q['radius']}"
              mass="{q['mass']}" rgba="0.8 0.8 0.85 1"/>
        <site name="tip" pos="0 0 {q['length']}" size="0.006"/>
      </body>"""

    if not compliant:
        # Motor + gearbox + rotor all lumped onto the cart joint.
        cart_joint = (
            f'<joint name="slider" type="slide" axis="1 0 0" limited="true" range="{-travel} {travel}" '
            f'armature="{reflected_rotor_mass(p)}" damping="{rail_damping}" '
            f'frictionloss="{rail_friction + gearbox_drag(p)}"/>'
        )
        drive_body = ""
        actuated_joint = "slider"
    else:
        cart_joint = (
            f'<joint name="slider" type="slide" axis="1 0 0" limited="true" range="{-travel} {travel}" '
            f'damping="{rail_damping}" frictionloss="{rail_friction}"/>'
        )
        # The drive body is massless except for the reflected rotor inertia (armature).
        drive_body = f"""
    <body name="drive" pos="0 -0.05 0">
      <joint name="drive" type="slide" axis="1 0 0" limited="false"
             armature="{reflected_rotor_mass(p)}" frictionloss="{gearbox_drag(p)}"/>
      <geom type="box" size="0.01 0.005 0.005" mass="1e-4" rgba="0.8 0.2 0.2 1" contype="0" conaffinity="0"/>
    </body>"""
        actuated_joint = "drive"

    return f"""<mujoco model="cart-pole single ({'compliant' if compliant else 'rigid'})">
  <!-- Generated by sim/model.py from sim/params.yaml. Edit params.yaml, not this file. -->
  <compiler angle="radian" inertiafromgeom="true"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <option timestep="{s['timestep']}" integrator="RK4" gravity="0 0 -9.81"/>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <light pos="0 -1 1.5" dir="0 1 -1"/>
    <camera name="side" pos="0 -1.05 0.02" xyaxes="1 0 0 0 0 1"/>
    <geom name="backdrop" type="box" pos="0 0.3 0" size="1.5 0.001 1" rgba="0.93 0.92 0.89 1"/>
    <geom name="rail" type="box" pos="0 0 -0.02" size="0.5 0.006 0.004" rgba="0.4 0.4 0.45 1"/>
    <geom name="end_left" type="box" pos="{-travel - 0.05} 0 -0.01" size="0.01 0.02 0.02" rgba="0.6 0.2 0.2 1"/>
    <geom name="end_right" type="box" pos="{travel + 0.05} 0 -0.01" size="0.01 0.02 0.02" rgba="0.6 0.2 0.2 1"/>
    <body name="cart" pos="0 0 0">
      {cart_joint}
      <geom name="cart" type="box" size="0.04 0.03 0.012" mass="{c['mass']}" rgba="0.15 0.35 0.25 1"/>
      {pole_body}
    </body>{drive_body}
  </worldbody>
  <actuator>
    <!-- DC motor, input = duty cycle in [-1, 1]. Force = G·duty − D·velocity (datasheet model). -->
    <general name="motor" joint="{actuated_joint}" ctrllimited="true" ctrlrange="-1 1"
             gainprm="{G}" biastype="affine" biasprm="0 0 {-D}"/>
  </actuator>
</mujoco>
"""


def double_xml(p: dict) -> str:
    """Double pendulum on the cart (rigid drive). Joint 2 sits at the top of rod 1 and carries its own
    bearings + sensor (the joint2 mass). hinge2 is measured RELATIVE to rod 1, like the real sensor."""
    c, q, j2, q2, s = p["cart"], p["pole"], p["joint2"], p["pole2"], p["sim"]
    travel = c["travel"]
    G, D = motor_gain(p), motor_damping(p)
    return f"""<mujoco model="cart-pole double">
  <!-- Generated by sim/model.py from sim/params.yaml. Edit params.yaml, not this file. -->
  <compiler angle="radian" inertiafromgeom="true"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <option timestep="{s['timestep']}" integrator="RK4" gravity="0 0 -9.81"/>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <light pos="0 -1 1.5" dir="0 1 -1"/>
    <camera name="side" pos="0 -1.3 0.1" xyaxes="1 0 0 0 0 1"/>
    <geom name="backdrop" type="box" pos="0 0.3 0" size="1.5 0.001 1" rgba="0.93 0.92 0.89 1"/>
    <geom name="rail" type="box" pos="0 0 -0.02" size="0.5 0.006 0.004" rgba="0.4 0.4 0.45 1"/>
    <geom name="end_left" type="box" pos="{-travel - 0.05} 0 -0.01" size="0.01 0.02 0.02" rgba="0.6 0.2 0.2 1"/>
    <geom name="end_right" type="box" pos="{travel + 0.05} 0 -0.01" size="0.01 0.02 0.02" rgba="0.6 0.2 0.2 1"/>
    <body name="cart" pos="0 0 0">
      <joint name="slider" type="slide" axis="1 0 0" limited="true" range="{-travel} {travel}"
             armature="{reflected_rotor_mass(p)}" damping="{c['viscous_friction']}"
             frictionloss="{c['coulomb_friction'] + gearbox_drag(p)}"/>
      <geom name="cart" type="box" size="0.04 0.03 0.012" mass="{c['mass']}" rgba="0.15 0.35 0.25 1"/>
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" limited="false" damping="{q['pivot_damping']}" frictionloss="{q.get('pivot_coulomb', 0.0)}" stiffness="{q.get('cable_stiffness', 0.0)}" springref="{math.pi}"/>
        <geom name="hub" type="sphere" size="0.012" mass="{q['hub_mass']}" rgba="0.3 0.3 0.3 1"/>
        <geom name="pole" type="cylinder" fromto="0 0 0 0 0 {q['length']}" size="{q['radius']}"
              mass="{q['mass']}" rgba="0.8 0.8 0.85 1"/>
        <!-- rod 2 sits 15 mm to the side of rod 1 so it can swing past it (needed for the UD and DU positions);
             an offset along the hinge axis doesn't change the motion in the plane of the swing -->
        <body name="pole2" pos="0 -0.015 {q['length']}">
          <joint name="hinge2" type="hinge" axis="0 1 0" limited="false" damping="{j2['damping']}" frictionloss="{j2.get('coulomb', 0.0)}"/>
          <geom name="joint2" type="sphere" size="0.01" mass="{j2['mass']}" rgba="0.3 0.3 0.3 1"/>
          <geom name="pole2" type="cylinder" fromto="0 0 0 0 0 {q2['length']}" size="{q2['radius']}"
                mass="{q2['mass']}" rgba="0.25 0.25 0.3 1"/>
          <site name="tip" pos="0 0 {q2['length']}" size="0.006"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general name="motor" joint="slider" ctrllimited="true" ctrlrange="-1 1"
             gainprm="{G}" biastype="affine" biasprm="0 0 {-D}"/>
  </actuator>
</mujoco>
"""


def build_double(p: dict | None = None, write: bool = False) -> mujoco.MjModel:
    p = p or load_params()
    xml = double_xml(p)
    if write:
        MODELS_DIR.mkdir(exist_ok=True)
        (MODELS_DIR / "double.xml").write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)


def build_single(p: dict | None = None, compliant: bool = False, write: bool = False) -> mujoco.MjModel:
    p = p or load_params()
    xml = single_xml(p, compliant)
    if write:
        MODELS_DIR.mkdir(exist_ok=True)
        (MODELS_DIR / ("single.xml" if not compliant else "single_compliant.xml")).write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)


if __name__ == "__main__":
    params = load_params()
    build_single(params, write=True)
    build_single(params, compliant=True, write=True)
    build_double(params, write=True)
    print(f"pulley radius      {pulley_radius(params) * 1000:.2f} mm")
    print(f"stall force        {motor_gain(params):.1f} N")
    print(f"back-EMF damping   {motor_damping(params):.1f} N·s/m")
    print(f"reflected rotor    {reflected_rotor_mass(params):.2f} kg (cart itself {params['cart']['mass']} kg)")
    print(f"gearbox drag       {gearbox_drag(params):.2f} N")
    print(f"backlash ±         {backlash_halfwidth(params) * 1000:.3f} mm")
    print("wrote models/single.xml and models/single_compliant.xml")
