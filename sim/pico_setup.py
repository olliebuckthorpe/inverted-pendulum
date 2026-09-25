"""Configure the Pico over USB (PLAN 2.5/5.1/6.1): every PARAM computed from sim/params.yaml and the designed
controllers, then both policies' weights. Nothing is hand-copied, so the Pico runs what the sim tested.

  uv run python pico_setup.py /dev/cu.usbmodemXXXX [--theta-zero RAD]
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time

import lqr
import link
import model
import swingup

CTRL_HZ = 500.0
SWING, BALANCE = "swingupE_seed3", "balanceGc_seed4"      # chosen by select_ai.py (96% hold-out)


def param_messages(p: dict, theta_zero: float = 0.0) -> list[tuple[int, float]]:
    """(id, value) pairs for every PARAM (ids: firmware/core/params.h)."""
    K = lqr.design(p, CTRL_HZ).ravel()
    c = swingup.EnergySwingUp(p, K.reshape(1, 4))
    sp = c.sp
    s = p["sensing"]
    r = model.pulley_radius(p)
    return [
        (1, 2 * math.pi * r / s["cart_counts_per_rev"]), (2, theta_zero), (3, s["velocity_filter_hz"]),
        (10, p["supply"]["voltage"]), (11, p["motor"]["R"]), (12, p["motor"]["Ke"]), (13, r),
        (14, p["supply"]["current_cap"] or 4.5),
        (20, K[0]), (21, K[1]), (22, K[2]), (23, K[3]), (24, 0.0),
        (25, c.m), (26, c.l), (27, c.J), (28, c.M_total), (29, c.G), (30, c.D), (31, c.F_coulomb),
        (32, sp.k_energy), (33, sp.a_max), (34, sp.k_x), (35, sp.k_v), (36, sp.nudge), (37, sp.target_margin),
        (38, math.radians(sp.catch_deg)), (39, sp.catch_energy), (40, math.radians(sp.drop_deg)),
        (50, math.radians(15)), (51, math.radians(35)), (52, 100.0), (53, 20.0),
        (60, 100.0), (61, 0.33), (62, 8.0), (63, 0.7), (64, 1.0), (65, 0.25),
    ]


def weight_floats(tag: str) -> list[float]:
    raw = (model.SIM_DIR.parent / "firmware" / "nn" / "weights" / f"{tag}.bin").read_bytes()
    assert raw[:4] == b"PPOL", "not a policy weights file"
    n = struct.unpack("<I", raw[8:12])[0]
    return list(struct.unpack(f"<{n}f", raw[16:16 + 4 * n]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--theta-zero", type=float, default=0.0, help="AS5047P reading (rad) with the rod straight up")
    a = ap.parse_args()
    p = model.with_params(model.load_params(), supply__current_cap=4.5)
    L = link.Link(a.port)
    L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
    msgs = [(link.PARAM, struct.pack("<Hf", i, float(v))) for i, v in param_messages(p, a.theta_zero)]
    msgs += [(link.WEIGHTS, m) for net, tag in ((0, SWING), (1, BALANCE)) for m in link.weights_messages(weight_floats(tag), net)]
    bad = 0
    for ptype, payload in msgs:
        L.send(ptype, payload)
        L.heartbeat()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            acks = [pl for t, _, pl in L.poll() if t == link.ACK]
            if acks:
                bad += acks[0][2] != 0
                break
    print(f"sent {len(msgs)} messages, {bad} rejected")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
