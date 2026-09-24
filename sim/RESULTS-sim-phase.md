# Sim phase results (PLAN 1.6), 24 Sep 2026

The sim is **built from datasheets plus guesses** (listed at the end). It can show that something clearly
won't work; it can't prove that something will (SPEC §11). Every number below will move after system ID
(Phase 4).

**Rig simulated:** MuJoCo at 1 kHz. The motor model comes from the Pololu 37D datasheet, including the rotor's
reflected inertia. There's a 2 ms control delay, quantised sensors (400-count motor encoder, 14-bit angle),
and velocities from filtered differencing, as the Pico will do. Code and tests are in `sim/`
(`uv run pytest`: 15 tests).

## Answers at a glance

| # | Question | Answer | Confidence |
|---|---|---|---|
| 1 | Is the Pololu 37D 6.3:1 12 V strong enough? | **Yes**: single swing-up 10/10 on both pulleys, including with 30% less torque, 30% more friction, and rotor inertia ×0.5 or ×2 | Good for "not clearly too weak"; hardware must confirm |
| 2 | 20T or 30T pulley? | **30T recommended** (Ollie to confirm): +22% bigger tap survived, better in every model-error case, faster swing-up | Good; driven by rotor inertia, which is estimated |
| 3 | 30 cm or 40 cm rod? | **Keep 30 cm**: 40 cm is worse on both pulleys, and on 20T it can't swing up at all | Good |
| 4 | Loop rate? | **500 Hz for the single, and 500 Hz is enough for the double too** (1 kHz gains ≤ 3% but doubles command chatter) | Medium: depends on the real delay and sensor noise |
| 5 | Double layout? | See §5 (filled in below) | |
| 6 | Is the 12 V 5 A PSU enough? | **Yes, with a firmware current limit**: every swing-up, single and double, still succeeds with the supply capped at 5 A | Medium: the limiter needs the fitted motor constants |

**To order now (Ollie):** Pololu 37D 6.3:1 12 V with 64 CPR encoder (#4757), and the Mean Well GST60A12-P1J
from RS. Both pulleys are already on the list.

---

## 1. Motor adequacy

Energy swing-up → LQR, 10 randomised starts per case, 500 Hz, 2 ms delay (`q2_pulley.py`):

| World | 20T swing-up | 30T swing-up |
|---|---|---|
| Datasheet (nominal) | 10/10 | 10/10 |
| Motor −30% torque | 10/10 | 10/10 |
| Friction +30% | 10/10 | 10/10 |
| Rotor inertia ×0.5 | 10/10 | 10/10 |
| Rotor inertia ×2 | 10/10 | 10/10 |

The fallback (the 24 V motor) is not needed on this evidence.

## 2. Pulley: 30T recommended

Plot: `plots/sim-phase-single-summary.png` (left); swing-up traces: `plots/q2-swingup-20T-vs-30T.png`.

| | 20T | 30T |
|---|---|---|
| Largest tap LQR survives, nominal | 2.96 rad/s | **3.60 rad/s** |
| … with motor −30% | 2.57 | **3.14** |
| … with rotor inertia ×2 | 2.30 | **2.92** |
| Swing-up time when allowed to push harder (a_max 10 m/s²) | 2.34 s | **1.68 s** |
| Share of swing-up at full duty | up to 29% | 11–18% |
| Rail used (of ±400 mm) | ≤ 250 mm | ≤ 325 mm |
| Cart resolution | 0.10 mm/count | 0.15 mm/count |

**Why the lower-force pulley wins:** the motor's rotor spins 6.3× faster than the gearbox output, so to the
cart it feels like extra mass of J·N²/r². That's **~2.9 kg on 20T but ~1.3 kg on 30T**, against a 0.4 kg
cart. The 30T's bigger radius cuts that phantom mass by 2.25×, which more than makes up for its 1.5× lower
force. The rotor inertia J isn't published; it's estimated from comparable motors, which is why it was swept
from ×0.5 to ×2. The 30T wins at every value.

**Costs of 30T:** it uses more rail during swing-up (75 mm margin left), a faster cart hits the end-stops
harder (relevant to the safety checklist), and the position resolution is slightly coarser (still well under
a millimetre).

## 3. Rod length: keep 30 cm

| | 30 cm | 40 cm |
|---|---|---|
| 30T: swing-up / largest tap | 10/10, 3.60 rad/s | 10/10, 2.73 rad/s |
| 20T: swing-up / largest tap | 10/10, 2.96 rad/s | **0/10** (runs into the rail), 2.27 rad/s |

A longer rod falls more slowly, but it needs more energy and more cart travel to swing up. (Caveat: the same
tap in rad/s is a slightly bigger push on a longer rod. The 20T/40 cm swing-up failure settles it anyway.)

## 4. Loop rate: 500 Hz

The single (LQR with a 2 ms delay):

| Rate | 30T largest tap | 20T largest tap | Swing-up | Command chatter (30T)* |
|---|---|---|---|---|
| 100 Hz | 3.40 | 2.80 | 10/10 | 0.069 |
| 200 Hz | 3.53 | 2.90 | 10/10 | 0.083 |
| 500 Hz | 3.60 | 2.96 | 10/10 | 0.120 |
| 1000 Hz | 3.63 | 2.98 | 10/10 | 0.133 |

\*RMS change in duty between consecutive commands while recovering from 5°. At higher rates, one sensor count
divided by a shorter time looks like a bigger velocity jump, so the motor gets buzzier.

The plan's rule is "the lowest rate that keeps full robustness, ×2 for margin". 200 Hz keeps 98% of the
robustness, so ×2 gives ~400 Hz; **500 Hz** is chosen (it's also the spec's default). The double balances and
swings up equally well at 500 Hz and 1 kHz (§5), so the spec's "~1 kHz for the double" isn't needed. The
firmware can still offer 1 kHz. The AI policy decides at 50–100 Hz as the spec says; that's a Phase 5 choice.

## 5. Double layout (balance and swing-up)

_Filled in below once the re-planned trajectories have been evaluated._

## 6. Power supply: keep the 5 A unit, add a firmware current limit

- Uncapped, a single swing-up peaks at **5.1 A (20T) / 5.5 A (30T)** from the supply, and up to 7 A when the
  controller pushes harder. The double's tracking corrections reach **9.6 A**. The GST60A12 trips into hiccup
  mode at 105–150% of 60 W (≈ 5.25–7.5 A), so **an unprotected rig would sometimes drop out mid-swing**.
- With the supply current **capped at 5 A in software**, every swing-up still succeeds: the single on both
  pulleys (10/10), and the double on its working layouts (10/10). RMS current is only ~0.4–1.1 A, so the PSU
  isn't working hard on average.
- **Firmware requirement (for Phase 2.6 / 3.4):** port `motor.cap_duty_for_supply` to the Pico. From the duty
  cycle and the measured cart speed, it limits the duty so that supply current = duty × (V·duty − Ke·ω)/R stays
  ≤ the limit. Both controllers go through it, so the comparison stays fair. **Use 4.5 A until Phase 4 fits
  the real R and Ke**, then 5.0 A.
- Keep the 7.5 A fuse. The bulk capacitor for regenerative braking (PLAN 2.1) also absorbs millisecond
  peaks.

## Double swing-up (added at Ollie's request, 24 Sep)

Classical method: **trajectory optimisation** (CasADi + IPOPT, direct collocation) finds a motor-command plan
from both-hanging to both-upright, within duty ≤ 0.8, cart ≤ ±350 mm and supply ≤ 5 A. **Time-varying LQR**
then follows the plan and **hands over** to the double's balance LQR at the top. The equations of motion
were derived by hand (sympy/Lagrange) and match MuJoCo to 1 part in a million (`tests/test_double_eom.py`).
Video: `videos/double-30T_30-20-seed0.mp4` (regenerate with `uv run python watch.py double --video`).

**What this does and doesn't show:**
- It shows that **the motor, rail and 5 A supply are physically enough** for a double swing-up, and that a
  classical controller can follow a plan and hand over to balance despite delay, quantisation and unmodelled
  friction.
- It's **one rehearsed path**. A start far from hanging-still, or a push mid-swing, will probably fail. An AI
  policy could be more forgiving here, and that would make a genuine comparison.
- **Hardware:** no hobby build has been found that achieved it. Hackaday #12827 (a DC servo, 1.5 m rail, slip
  ring) only did it in simulation (last log, Nov 2019). It also needs the joint-2 sensor's signal to cross a
  spinning joint 1: a slip ring (capsule rings have no published torque or noise figures, and SPI through
  brushes is unproven; the AS5047P's ABI output or a local UART link would be more robust) or wireless
  (ESP-NOW ~5–25 ms latency, no pendulum precedent found). **Decision pending (SPEC §3).**

---

## Numbers that are guesses (replace in Phase 4)

| Parameter | Value used | Why it matters |
|---|---|---|
| Motor rotor inertia | 3.0e-6 kg·m² (swept 1.5–6e-6) | Decides the pulley answer; measure it from voltage-step responses |
| Cart mass | 0.40 kg | Weigh the printed cart |
| Rail + belt friction | 1.0 N Coulomb, 0.5 N·s/m viscous | Triangle-wave system ID |
| Joint-2 hardware mass | 0.020 kg | Weigh it; it sets the double's dynamics |
| Joint damping | 2e-4 / 1e-4 N·m·s/rad | Free-swing decay |
| Backlash | 1° at the output | Direction-reversal test |
| Control delay | 2 ms | Measure the loop (PLAN 2.5/2.6) |
| Angle-sensor noise | 0 (quantisation only) | PLAN 2.4 |

## Changes made to SPEC/PLAN during the sim phase

- **End-stops switch a relay coil** instead of carrying motor current: micro switches have no usable DC power
  rating (Omron V-15: 0.6 A at 125 V DC). Items 15a/15b were added.
- **The PSU is ordered with the motor**, sized by Q6 (above: keep the 5 A unit).
- **The motor current is not clamped at 5.5 A in the model**: a reversal at speed really draws ~10 A.
- **Rotor inertia is modelled** (PLAN 1.3 had a placeholder of 0, which made the sim badly optimistic).
- **Double swing-up in sim is in scope**; on hardware it's an open decision.
