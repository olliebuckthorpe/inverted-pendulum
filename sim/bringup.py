"""Bench bring-up checks (PLAN 2.4, 2.6, 3.2–3.4), one command each; hardware/BRINGUP.md says when to run them.
Every command first sends the PARAMs (as pico_setup.py does), keeps the heartbeat going, and leaves the Pico in
IDLE (motor braked) when it ends or is interrupted.

  uv run python bringup.py PORT watch                 # live: mode, cart mm, rod angle, duty, faults, live inputs, loop timing
  uv run python bringup.py PORT calibrate             # rod hanging still: saves θ's zero (upright) for every later command
  uv run python bringup.py PORT noise [--seconds 10]  # 2.4: angle noise with the rod still -> analysis/sensor-noise.md
  uv run python bringup.py PORT ruler                 # 3.3: cart distance vs a ruler (counts per metre within 1 %)
  uv run python bringup.py PORT loop [--minutes 10]   # 2.6: loop execution time and jitter -> analysis/loop-timing.md
  uv run python bringup.py PORT spin --duty 0.10 --ms 300   # a short low-duty jolt (wiring, E-stop and relay tests)
  uv run python bringup.py PORT levels                # 3.2: hold PWM, then DIR, steadily HIGH to measure ≥ 3.0 V (motor DISCONNECTED)
  uv run python bringup.py PORT hang                  # 3.4: deliberate hang -> hardware watchdog reset -> boots braked
  uv run python bringup.py PORT fallcheck             # 3.4: balance mode with the rod HANGING must fault at once, no motion

PORT is the Pico's USB serial port, e.g. /dev/cu.usbmodem1101 (`ls /dev/cu.usbmodem*`).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import struct
import sys
import time

import numpy as np

import link
import model
import pico_setup

PERIOD_US = 2000                     # the firmware's 500 Hz loop
CALIBRATION = pico_setup.CALIBRATION
theta_zero = pico_setup.theta_zero


def zero_from_hanging(th_hanging: np.ndarray, th0_now: float) -> float:
    """New θ zero from readings taken with the rod hanging still: the telemetry θ is wrap(raw − θ0), and upright
    is the hanging reading + π. Circular mean, so a hanging reading near ±π doesn't average to 0."""
    th = np.asarray(th_hanging, float)
    raw = math.atan2(np.sin(th).mean(), np.cos(th).mean()) + th0_now
    return math.atan2(math.sin(raw + math.pi), math.cos(raw + math.pi))


# ---- pure helpers (unit-tested in tests/test_bringup.py) -----------------------------------------------------
def noise_stats(th: np.ndarray) -> dict:
    """Angle noise with the rod still: std-dev and peak-to-peak in degrees, and the std-dev after averaging
    N consecutive readings (PLAN 2.4's averaging test: white noise falls as 1/sqrt(N); drift or quantisation
    doesn't)."""
    deg = np.degrees(np.unwrap(np.asarray(th, float)))
    deg = deg - deg.mean()
    out = {"n": int(deg.size), "std_deg": float(deg.std()), "p2p_deg": float(np.ptp(deg))}
    for n in (2, 4, 8):
        m = deg[: deg.size // n * n].reshape(-1, n).mean(axis=1)
        out[f"std_avg{n}_deg"] = float(m.std())
    return out


def ruler_error(x_start_m: float, x_end_m: float, ruler_mm: float) -> float:
    """Percentage error of the encoder's distance against the ruler (PLAN 3.3: within 1 %)."""
    return 100.0 * (abs(x_end_m - x_start_m) * 1000.0 - ruler_mm) / ruler_mm


def loop_stats(exec_us: np.ndarray, jitter_us: np.ndarray) -> dict:
    """PLAN 2.6: max execution time below 50 % of the 2000 µs period."""
    e, j = np.asarray(exec_us, float), np.asarray(jitter_us, float)
    return {"frames": int(e.size), "exec_mean_us": float(e.mean()), "exec_p99_us": float(np.percentile(e, 99)),
            "exec_max_us": float(e.max()), "jitter_mean_us": float(j.mean()), "jitter_max_us": float(j.max()),
            "pass": bool(e.max() < 0.5 * PERIOD_US)}


def status_line(t: dict) -> str:
    faults, live = link.decode_faults(t["faults"])
    mode = {v: k for k, v in link.MODES.items()}.get(t["mode"], str(t["mode"]))
    ins = "  ".join(f"{'✓' if v else '✗'} {k}" for k, v in live.items())
    return (f"{mode:9s} x {t['x'] * 1000:7.1f} mm  θ {math.degrees(t['th']):7.2f}°  duty {t['duty']:+.2f}  "
            f"exec {t['exec_us']:4d} µs  jitter {t['jitter_us']:3d} µs | {ins} | faults: {', '.join(faults) or 'none'}")


# ---- talking to the Pico ------------------------------------------------------------------------------------
def configure(L) -> None:
    """IDLE, then every PARAM from params.yaml (x per count, θ zero, limits…), waiting for each ACK."""
    p = model.with_params(model.load_params(), supply__current_cap=4.5)
    L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
    for i, v in pico_setup.param_messages(p, theta_zero()):
        L.send(link.PARAM, struct.pack("<Hf", i, float(v)))
        L.heartbeat()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not any(t == link.ACK for t, _, _ in L.poll()):
            time.sleep(0.002)


def telemetry(L, seconds: float | None = None):
    """Yield decoded telemetry frames for `seconds` (None = until Ctrl-C), heartbeating every 20 ms."""
    end = None if seconds is None else time.monotonic() + seconds
    beat = 0.0
    while end is None or time.monotonic() < end:
        now = time.monotonic()
        if now - beat > 0.02:
            L.heartbeat()
            beat = now
        for ptype, _, payload in L.poll():
            if ptype == link.TELEMETRY and len(payload) == struct.calcsize(link.TELEMETRY_FMT):
                yield link.unpack_telemetry(payload)
            elif ptype == link.TEXT:
                print("Pico:", payload.decode(errors="replace"))
        time.sleep(0.001)


def record(L, seconds: float) -> dict:
    rows = list(telemetry(L, seconds))
    return {k: np.array([r[k] for r in rows]) for k in rows[0]} if rows else {}


def _write(relpath: str, text: str) -> None:
    path = model.SIM_DIR.parent / relpath
    path.parent.mkdir(exist_ok=True)
    with path.open("a") as f:
        f.write(text)
    print(f"appended to {relpath}")


# ---- commands -----------------------------------------------------------------------------------------------
def cmd_watch(L, a):
    last = 0.0
    for t in telemetry(L):
        if time.monotonic() - last > 0.1:
            print("\r" + status_line(t), end="", flush=True)
            last = time.monotonic()


def cmd_calibrate(L, a):
    input("Rod 1 hanging straight down and still (no rod 2). Press Enter to record 2 s… ")
    th0_now = theta_zero()
    r = record(L, 2.0)
    new = zero_from_hanging(r["th"], th0_now)
    CALIBRATION.write_text(json.dumps({"theta_zero_rad": new, "date": str(dt.date.today()),
                                       "how": "bringup.py calibrate: rod hanging still, upright = hanging + 180°"}, indent=2) + "\n")
    print(f"θ zero = {new:.4f} rad ({math.degrees(new):.2f}°) saved to hardware/rig-calibration.json; "
          f"commit it. The hanging rod now reads ±180°.")
    configure(L)


def cmd_noise(L, a):
    input(f"Hold the rod still (hanging, untouched), then press Enter to record {a.seconds:.0f} s… ")
    r = record(L, a.seconds)
    ok = r["faults"] >> 15 & 1
    st = noise_stats(r["th"][ok == 1])
    print(f"{st['n']} readings ({int((ok == 0).sum())} failed reads): std {st['std_deg']:.4f}°, "
          f"peak-to-peak {st['p2p_deg']:.4f}°; averaged over 2/4/8: {st['std_avg2_deg']:.4f} / "
          f"{st['std_avg4_deg']:.4f} / {st['std_avg8_deg']:.4f}°  (datasheet 0.068° RMS)")
    _write("analysis/sensor-noise.md",
           f"\n## {dt.date.today()} — AS5047P, rod still, {a.seconds:.0f} s at 500 Hz (`bringup.py noise`)\n\n"
           f"| readings | failed reads | std-dev | peak-to-peak | avg of 2 | avg of 4 | avg of 8 |\n|---|---|---|---|---|---|---|\n"
           f"| {st['n']} | {int((ok == 0).sum())} | {st['std_deg']:.4f}° | {st['p2p_deg']:.4f}° | {st['std_avg2_deg']:.4f}° | "
           f"{st['std_avg4_deg']:.4f}° | {st['std_avg8_deg']:.4f}° |\n\nDatasheet: 0.068° RMS. Put the std-dev into "
           f"`sim/params.yaml` sensing.angle_noise_deg.\n")


def cmd_ruler(L, a):
    def x_now():
        xs = [t["x"] for t in telemetry(L, 0.3)]
        return float(np.median(xs))
    input("Line the cart up with a mark (e.g. 0 on a steel rule taped to the beam) and press Enter… ")
    x0 = x_now()
    mm = float(input("Push the cart by hand to another mark. How far did it move on the ruler (mm)? "))
    x1 = x_now()
    err = ruler_error(x0, x1, mm)
    print(f"encoder says {abs(x1 - x0) * 1000:.1f} mm, ruler {mm:.1f} mm: error {err:+.2f} % "
          f"({'PASS' if abs(err) <= 1.0 else 'FAIL'}: PLAN 3.3 needs within 1 %)")


def cmd_loop(L, a):
    print(f"recording loop timing for {a.minutes:g} min (IDLE; run it again in CLASSICAL once safe)…")
    r = record(L, a.minutes * 60)
    st = loop_stats(r["exec_us"], r["jitter_us"])
    print(f"{st['frames']} frames: exec mean {st['exec_mean_us']:.0f} / p99 {st['exec_p99_us']:.0f} / max "
          f"{st['exec_max_us']:.0f} µs, jitter mean {st['jitter_mean_us']:.1f} / max {st['jitter_max_us']:.0f} µs: "
          f"{'PASS' if st['pass'] else 'FAIL'} (max exec < 1000 µs)")
    _write("analysis/loop-timing.md",
           f"\n## {dt.date.today()} — {a.minutes:g} min, mode IDLE (`bringup.py loop`)\n\n"
           f"| frames | exec mean | exec p99 | exec max | jitter mean | jitter max | < 50 % of 2 ms |\n|---|---|---|---|---|---|---|\n"
           f"| {st['frames']} | {st['exec_mean_us']:.0f} µs | {st['exec_p99_us']:.0f} µs | {st['exec_max_us']:.0f} µs | "
           f"{st['jitter_mean_us']:.1f} µs | {st['jitter_max_us']:.0f} µs | {'yes' if st['pass'] else 'NO'} |\n")


def cmd_spin(L, a):
    if abs(a.duty) > 0.3:
        sys.exit("refusing |duty| > 0.3 for a bench jolt")
    if a.no_soft_limit:                      # safety-checklist item 2: let the cart reach the end-stops
        L.send(link.PARAM, struct.pack("<Hf", 61, 0.45))            # X_SOFT beyond the rail (end-stops ~0.348 m)
    L.send(link.SET_MODE, bytes([link.MODES["KICK"]]))
    L.send(link.KICK, struct.pack("<fH", a.duty, a.ms))
    for t in telemetry(L, a.ms / 1000 + 0.5):
        print(status_line(t))
    if a.no_soft_limit:
        L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
        configure(L)                         # the soft limit back to 0.33 m


def cmd_levels(L, a):
    """hardware/electronics.md logic-level check: the MDD10A needs >= 3.0 V HIGH. A multimeter can't read a
    20 kHz PWM, so hold 100 % duty (PWM steadily HIGH), then -100 % (DIR HIGH too), with the supply cap off."""
    if input("Motor leads DISCONNECTED from the driver/relay, E-stop released, end-stops closed? Type yes: ") != "yes":
        sys.exit("stopped: disconnect the motor first")
    L.send(link.PARAM, struct.pack("<Hf", 14, 0.0))            # SUPPLY_CAP_A = 0: no cap, so duty 1.0 is a steady HIGH
    for duty, what in ((1.0, "PWM1 (GP14)"), (-1.0, "DIR1 (GP15) — PWM stays HIGH")):
        input(f"Press Enter, then measure {what} to GND at the MDD10A connector within 8 s (need >= 3.0 V)… ")
        L.send(link.SET_MODE, bytes([link.MODES["KICK"]]))
        L.send(link.KICK, struct.pack("<fH", duty, 8000))
        for t in telemetry(L, 8.3):
            pass
        faults, _ = link.decode_faults(t["faults"])
        print(f"  held duty {t['duty']:+.2f}; faults: {', '.join(faults) or 'none'}")
        L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
    configure(L)                                                  # restore every PARAM, incl. the supply cap


def cmd_hang(L, a):
    """PLAN 3.4: the hardware watchdog (50 ms) must reset a hung Pico, which boots with the motor braked.
    A reset drops the USB serial port, so a lost port counts as the reset; then reconnect and read the boot."""
    L.send(link.HANG)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < 2.0:
            for ptype, _, payload in L.poll():
                if ptype == link.ACK and payload[2] != 0:
                    sys.exit("refused (the Pico only accepts HANG in IDLE)")
            time.sleep(0.005)
        print("FAIL: the port is still up 2 s after HANG, so the Pico did not reset. Unplug it now.")
        return
    except OSError:                                              # serial.SerialException is an OSError
        print(f"USB dropped {time.monotonic() - t0:.2f} s after HANG: the Pico reset. Reconnecting…")
    for _ in range(50):
        time.sleep(0.1)
        try:
            L2 = link.Link(a.port)
            break
        except OSError:
            continue
    else:
        sys.exit("the port didn't come back within 5 s: check `ls /dev/cu.usbmodem*` and run `watch`")
    configure(L2)
    t = next(telemetry(L2, 2.0), None)
    ok = t is not None and t["mode"] == link.MODES["IDLE"] and t["duty"] == 0.0
    print(f"{'PASS' if ok else 'FAIL'}: after the reset the Pico reports mode "
          f"{'IDLE' if t and t['mode'] == 0 else t and t['mode']}, duty {t['duty'] if t else '?'} (motor braked)")


def cmd_fallcheck(L, a):
    """PLAN 3.4 fall detection, without risking a balance attempt: LQR_ONLY with the rod hanging (θ ≈ 180°, far
    past the 40° fall limit) must latch 'fell' on the first cycle and never drive the motor."""
    input("Rod hanging still, cart mid-rail, hands clear. Press Enter… ")
    L.send(link.SET_MODE, bytes([link.MODES["LQR_ONLY"]]))
    rows = [t for t in telemetry(L, 1.0) if t["mode"] == link.MODES["LQR_ONLY"]]
    L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
    fell = [bool(t["faults"] >> 2 & 1) for t in rows]
    peak = max((abs(t["duty"]) for t in rows), default=float("nan"))
    ok = bool(rows) and all(fell) and peak == 0.0
    print(f"{'PASS' if ok else 'FAIL'}: {len(rows)} frames in LQR_ONLY, 'fell' latched in {sum(fell)}, peak duty {peak:.3f}")


def main():
    if hasattr(sys.stdout, "reconfigure"):          # Windows consoles default to cp1252, which can't print ✓/✗/θ
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("watch")
    sub.add_parser("calibrate")
    sub.add_parser("noise").add_argument("--seconds", type=float, default=10.0)
    sub.add_parser("ruler")
    sub.add_parser("loop").add_argument("--minutes", type=float, default=10.0)
    sp = sub.add_parser("spin")
    sp.add_argument("--duty", type=float, default=0.10)
    sp.add_argument("--ms", type=int, default=300)
    sp.add_argument("--no-soft-limit", action="store_true", help="safety test 2 only: drive into an end-stop")
    sub.add_parser("levels")
    sub.add_parser("hang")
    sub.add_parser("fallcheck")
    a = ap.parse_args()
    L = link.Link(a.port)
    try:
        configure(L)
        {"watch": cmd_watch, "calibrate": cmd_calibrate, "noise": cmd_noise, "ruler": cmd_ruler, "loop": cmd_loop, "spin": cmd_spin,
         "levels": cmd_levels, "hang": cmd_hang, "fallcheck": cmd_fallcheck}[a.cmd](L, a)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            L.send(link.SET_MODE, bytes([link.MODES["IDLE"]]))
            print("\nPico left in IDLE (motor braked).")
        except OSError:
            pass                                                  # the port went away (e.g. after `hang`)


if __name__ == "__main__":
    main()
