# SPEC — Classical Control vs AI: an Inverted Pendulum on a Cart

Written 2026-09-24. Owner: Ollie Buckthorpe. Status: **agreed spec, no code, nothing bought yet.**

This is the contract. Every later session builds from this file. If something here turns out to be wrong,
change this file first, then the plan, then the code. Decisions were made in question rounds with Ollie and are
evidence-based; sources are listed in §17.

---

## 1. One line

A self-built inverted pendulum on a belt-driven cart, balanced and swung up **two ways**, with classical
control (energy-pumping swing-up + LQR) and with AI (two SAC policies trained in MuJoCo), then compared
head-to-head on the same real hardware using a test set fixed in advance.

## 2. Why

- **The Academy application** (theacademysf.com), **deadline 5 November 2026**.
- The portfolio already shows software and traction (Upshow), business (Young Enterprise: £1,600 revenue)
  and physical making (a 3D-printed F1 wing). It lacks **hard engineering**: electronics, maths and control.
  This project fills that gap and brings in AI.
- **It must look finished on the application.** Pitch it as a complete study ("classical vs AI on real
  hardware"), never as "stage 1 of 3".

## 3. Scope

| Tier | What | Status by 5 Nov |
|---|---|---|
| **Must have** | **Single rod**: classical and AI both swing up and balance on the real rig; the full test set is run; filmed | Required |
| **Stretch** | **Double rod**: balances on the real rig (started upright by hand). LQR on hardware, with AI in simulation and possibly on hardware. **Double swing-up + hand-over to balance in simulation** (added 2026-09-24 at Ollie's request; classical = trajectory optimisation + time-varying LQR). **Transitions between all four resting positions (DD, UU, UD, DU) in simulation**, all 12 working (added 2026-09-24); on hardware, they come after the double swing-up, behind the same gates. **Double swing-up on hardware: Ollie decided to attempt it (24 Sep)**, see below | Only after the single is done. **Hard stop 1 Nov** |
| **Reach** | Triple | **Out of scope before 5 Nov.** "What's next" in the write-up |

**AI for the double (added 2026-09-30 at Ollie's request):** the AI double balance in sim is done (97–100/100
on realistic rigs, `sim/RESULTS-ai-double.md`); an **AI double swing-up in sim is an exploratory experiment**,
reported honestly whether or not it learns. Neither changes the hardware order: classical double first.

**Double swing-up on hardware (decided 2026-09-24: attempt it, as the top of the stretch tier).** The sim
shows it's physically possible with this motor, rail and PSU (sim/RESULTS-sim-phase.md), but no hobby build
has been found that achieved it (the closest, Hackaday #12827, only did it in simulation). It stays behind the
same gates: the single must be done first, then double balance, and **the double still stops on 1 Nov**. If
the swing-up isn't working by then, the result is "double balances on hardware; swing-up shown in sim".
What it needs beyond double balance (all found in the sim, 24 Sep; sim/RESULTS-sim-phase.md):
- **A Kalman filter** on the Pico for the double (not plain differencing). With the AS5047P's datasheet noise
  (0.068° RMS), differencing fails 0/10 and the Kalman filter succeeds 10/10. Its gains are precomputed on
  the Mac, so the Pico only does small matrix-vector products.
- **A "gentle, friction-margin" plan:** duty ≤ 0.6, cart within ±300 mm, planned assuming **1.5× the
  measured joint-2 friction**. That plan swings up on 50/50 random rigs at realistic post-system-ID errors,
  and 48/50 at twice those errors.
- **The double's masses weighed and its joint friction measured** (free-swing decay), then the plan
  recomputed on the fitted model (~10 min).
- **Getting the joint-2 signal across joint 1: a flexible cable loop, not a slip ring** (decided on
  evidence, 24 Sep). In the plan and in 20 flights on random rigs, rod 1 stays within −6°…251° (at most half a
  turn from hanging), so nothing has to spin continuously. Slip rings were rejected: SenRing specifies its
  capsule rings at 0.05 N·m (+0.01 N·m per 6 circuits), about 25× the 2 mN·m of drag the swing-up tolerates;
  The Pi Hut's ring (£15.20) publishes no torque at all. The cable must twist softly: **≤ 0.5 mN·m/rad** if
  unmodelled (the sim fails at 1.0). Use ultra-flexible stranded silicone wire (e.g. 30 AWG) in a long loop;
  ABI needs only 4–5 wires. Bench-measure the stiffness (PLAN 2.5b). The firmware counts rod-1 turns and cuts
  the motor beyond ±1 turn, to protect the cable.

**Out of scope:** triple swing-up on hardware, a custom PCB, running the AI on the Mac
in the loop, Isaac Sim, and hand-derived equations (optional later).

## 4. Definition of done

**The single is done when all of these are true:**
1. Classical (energy swing-up → LQR) swings up from hanging and balances on the real rig.
2. The AI (swing-up policy → hand-over → balance policy) swings up and balances on the real rig.
3. The full test set in §12 has been run for both, with the raw data logged.
4. The scorecard chart exists.
5. Every milestone has been filmed, and a paragraph written on the same day.

**Gate:** the double starts only after the single is done. **The double stops on 1 Nov** whatever state it's in.
2–5 Nov are for the videos, the project page, the README and submission.

## 5. System overview

```
 Mac (Python)                                   Pico 2 (Arduino C++)                  Rig
 ─────────────                                  ────────────────────                  ───
 MuJoCo sim + motor model                       fixed-rate control loop               Pololu 37D motor + encoder ──► belt ──► cart on MGN12 rail
 System ID (fit sim to recordings)   USB CDC    ├─ read motor encoder (PIO)  ◄──────  (cart position)
 LQR design (python-control/scipy)  ◄───────►   ├─ read AS5047P (SPI)        ◄──────  magnet on joint 1 shaft (rod angle)
 SB3 SAC training (Mac/Windows)     binary+COBS ├─ controller: LQR | energy | NN
 Export gains + NN weights ────────────────►    ├─ safety: watchdog, soft limits
 Test runner + logging ◄── telemetry ───────    └─ PWM + DIR ────────────────────►   Cytron MD13S ──► motor
 PlotJuggler (live)                                                                   12 V PSU → fuse → E-stop + end-stops → driver
```

**Key architectural decision: the control loop runs entirely on the Pico**, for both controllers. The Mac
designs and trains, then sends numbers (LQR gains, NN weights) to the Pico, and receives telemetry. Evidence:
ferrolho's pendulum ran a 689-parameter NN on an Arduino Nano; Re-Forward ran its policy on an STM32; the only
PC-in-the-loop double (Hackaday #12827) needed special networking code to reach 2 kHz. The Pico 2 (RP2350) has a
hardware FPU.

## 6. Hardware

Prices were checked 2026-09-24. ✅ = confirmed on the seller's page; ⚠️ = search snippet or unconfirmed, so
**re-check at checkout**. **Order everything except the motor now; order the motor after the sim phase (§11).**

**Ordered 28–29 Sep 2026.** What was actually bought (sellers, part numbers, prices) is in `hardware/orders.md`.
Four parts were substituted at checkout, each for stock or price, with the evidence there and in
`hardware/electronics.md`: the **MDD10A** driver (row 8), the **Pico 2 W** (row 6) and the **1N4001** (15b). The
power supply, sensors and small parts came from **DigiKey** (1 Oct): the RS order never completed, and the
AS5047P kit is now discontinued (3 bought: 1 spare). Rows 13 (rod) and the frame (plywood, clamps, wood screws) are bought in person when needed.

| # | Part | Spec to match | Qty | Seller (UK) | Price | Buy |
|---|---|---|---|---|---|---|
| 1 | Linear rail | **MGN12, 1000 mm, with 1× MGN12H carriage** (clone OK) | 1 | eBay UK, CNC-Planet | ⚠️ £11.95 base; **1000 mm price unconfirmed** (~£12–20) | Now |
| 2 | Timing belt | GT2, **6 mm wide**, ~2.5 m | 1 | Technobots | ✅ £4.63 | Now |
| 3 | Drive pulley | GT2 **20T, 6 mm bore**, 6 mm belt | 1 | Gears & Sprockets | ✅ £1.78 | Now |
| 4 | Drive pulley | GT2 **30T, 6 mm bore**, 6 mm belt | 1 | Gears & Sprockets | ✅ £2.63 | Now |
| 5 | Idler | GT2 20T idler, 5 mm bore (or bearings for a printed idler) | 1 | Technobots | ⚠️ ~£2.02 | Now |
| 6 | Controller | **Raspberry Pi Pico 2 H** (with headers). *Ordered: Pico 2 W with headers, run as a Pico 2 (radio off)* | 1 | The Pi Hut | ✅ £5.80 | Now |
| 7 | Angle sensor | **ams-OSRAM AS5047P-TS_EK_AB** adapter kit (14-bit SPI; **includes a 6×2.5 mm diametric magnet**) | 1 now, +1 for the double | Farnell / RS / Mouser UK | ⚠️ ~£13–17 (Digi-Key $16.66) | Now (2nd later) |
| 8 | Motor driver | **Cytron MD13S** (6–30 V, 13 A, 3.3 V logic OK). *Ordered: Cytron MDD10A (5–30 V, 10 A per channel, same PWM/DIR interface and truth table), channel 1 only* | 1 | Active Robots (or RobotBits) | ⚠️ ~£15 (VAT status unclear) | Now |
| 9 | Power supply | **Mean Well GST60A12-P1J**, 12 V 5 A, enclosed (**or a larger 12 V unit if the sim says so**, see §8) | 1 | **RS UK** (sold out at The Pi Hut) | ✅ £25.91 inc VAT | **With the motor, after the sim phase** |
| 10 | Barrel jack adapter | 2.1 mm DC jack → screw terminals | 1 | Pi Hut / Amazon | ⚠️ ~£2–3 | Now |
| 11 | Bearings | 608ZZ (8 mm bore), pack of 10 | 1 pack | Amazon UK | ✅ £5.58 | Now |
| 12 | Joint shaft | 8 mm silver steel, single cut length ~150–300 mm | 1 | eBay UK (not RS: 3 m minimum) | ⚠️ ~£5–6 | Now |
| 13 | Rod | **Aluminium tube, 8 mm OD, 1 m** (cut to 30 cm, with spare for a 40 cm) | 1 | Wickes | ✅ £4.20 | Now |
| 14 | E-stop | Latching mushroom switch, ≥10 A DC | 1 | Amazon UK | ⚠️ ~£8.49–9.99 | Now |
| 15 | End-stops | Micro limit switch, roller lever, pack of 10 (**signal/relay-coil duty only**, see §8) | 1 pack | Amazon UK | ✅ £6.99 | Now |
| 15a | Brake relay | **12 V automotive CHANGEOVER relay, 5-pin (30 / 87 / 87a), 30–40 A contacts**, with socket/pigtail. In the motor lead: drops → shorts the motor (hardware/electronics.md) | 1 | Amazon UK / car-parts shop | ⚠️ ~£4–6 | Now |
| 15b | Flyback diode | 1N4007 (across the relay coil). *Ordered: 1N4001, the 50 V grade of the same diode* | 1 pack | Amazon UK | ⚠️ ~£1–2 | Now (skip if owned) |
| 15c | Schottky diode | ≥ 8 A, ≥ 40 V (e.g. MBR1045, TO-220): stops braking energy reaching the PSU (hardware/electronics.md) | 1 | Any UK electronics seller | ⚠️ ~£1 | Now |
| 15d | Bulk capacitor | 2200 µF 35 V low-ESR electrolytic, at the driver | 1 | Any UK electronics seller | ⚠️ ~£1–2 | Now |
| 15e | TVS diode | ~21–23 V breakdown, unidirectional (e.g. 1.5KE22A). *Ordered: 1.5KE22A (DigiKey)* | 1 | Any UK electronics seller | ⚠️ ~£0.50 | Now |
| 15f | Resistors | 10 kΩ ×5, 3.3 kΩ ×3, 1 kΩ ×2 (pull-downs, end-stop dividers, encoder series) | – | Any UK electronics seller | ⚠️ ~£1–5 | Now (skip if a kit is owned) |
| 16 | Fuse | Inline blade fuse holders + mixed fuses (use **7.5 A**) | 1 kit | Amazon UK | ✅ £7.99 | Now |
| 17 | Wiring | 18 AWG silicone red/black; Dupont jumpers; small breadboard | – | Amazon UK | ✅ £6.74 / ✅ £6.99 / ⚠️ ~£7.99 | Now (skip anything already owned) |
| 18 | Fasteners | M3/M5 bolt and nut assortment | 1 | Amazon UK | ✅ £7.99 | Now (skip if owned) |
| 19 | **Motor** | **Pololu 37D, 6.3:1, 12 V, with 64 CPR encoder (helical pinion)**: 1600 RPM, 0.294 N·m stall, 5.5 A stall, 400 CPR at output, 6 mm D-shaft | 1 | Cheapest UK/EU source (Pololu list $60.95; Opencircuit €82.30) | ⚠️ ~£55–70 | **After the sim phase** |
| 20 | Tapper servo | Small hobby servo (for the push tapper) | 1 | – | ⚠️ ~£3–5 | During the build |
| 21 | Top rod (double) | Carbon fibre tube, ~8 mm | 1 | eBay UK | ⚠️ ~£5–15 | Only for the double |

### Budget (honest)

| | Low | High |
|---|---|---|
| Items 1–18 + 15a/15b (order now; PSU moves to the motor order) | ~£158 | ~£179 |
| Motor (item 19) | ~£55 | ~£70 |
| Servo (item 20) | ~£3 | ~£5 |
| **Upfront total** | **~£211** | **~£246** |
| Later, for the double: 2nd AS5047P + carbon rod | +~£18 | +~£32 |

⚠️ **This is over the £150–200 target, by about £16–54** (the relay adds ~£5–8; a bigger PSU, if the sim needs one, would add ~£10–20 more). Ways to close the gap, in order:
1. **Skip what's already owned** (breadboard, jumpers, bolts, wire): up to ~£30.
2. **Find the cheapest Pololu seller**: ~£5–15.
3. Print the idler around bearings already in the 608 pack. The 3D printer also replaces every bracket.

Do **not** save money on the rail, motor or sensor. Those are where precision decides whether it works.

### Why each major part (evidence summary)

- **Motor:** every documented double used a DC servo or linear motor, never an open-loop stepper. The only
  documented open-loop stepper cart was jerky. The Pololu has documented specs; its gearbox play is unpublished
  (a Pololu forum post says "expect a couple of degrees"; a customer measured ~1°).
- **Rail:** MGN12 was used in the RunTheBot double. V-slot builds reported friction (ZIPY). Rods sag at 1 m.
- **Sensor:** single builds worked at 0.15–0.25°/count; the documented double used 0.018°. The AS5047P gives
  0.022°, uses SPI (fast), and the kit includes the correct magnet. The MT6701 was rejected: no UK-stock board
  exposes SSI. The AS5600 was rejected: I2C only, 0.088°, and there's a forum report of DC-motor noise.
- **Pico 2:** has an FPU, PIO for jitter-free encoder decoding, and confirmed support in the Arduino-Pico core.
- **Driver:** the Cytron accepts 3.3 V logic. The BTS7960 was rejected (uncertain at 3.3 V) and the L298N
  (a ~2 V voltage drop).
- **PSU:** an enclosed brick is beginner-safe. The PC's ATX supply was rejected (it's in a working PC, needed
  for weeks, and the saving was only ~£10–15). **Added 2026-09-24:** the GST60A12's overload protection trips at
  105–150% of rated power into **hiccup mode** (Mean Well GST60A spec), i.e. ~5.25–7.5 A, while the motor's
  stall current is 5.5 A and a full-speed reversal can draw ~10 A. So the sim logs supply current and decides
  whether the 5 A unit is enough (§11 Q6); the PSU is ordered with the motor.
- **Relay for the end-stops (added 2026-09-24):** micro switches are AC-rated. Omron's V-15 is 15 A at 250 V AC
  but only 0.6 A at 125 V DC, with no 12 V DC rating; only the special V-21(IN) is rated 12 A at 30 V DC (Omron
  V-series datasheet). DC arcs don't self-extinguish, so a micro switch breaking 5–10 A can weld shut and fail
  unsafe. The end-stops therefore switch only a relay coil (~0.15 A), and an automotive relay, designed for 12 V
  DC loads, breaks the motor current.

## 7. Mechanical design

- **Rail:** 1 m MGN12, bolted flat to a rigid base, with printed end blocks holding the motor and idler.
- **Cart:** 3D-printed and **as light as possible** (less mass means less force needed). It carries joint 1
  and its sensor.
- **Drive:** GT2 6 mm belt, a metal pulley on the motor, and the idler at the far end. **30T chosen
  (Ollie, 24 Sep), from the sim** (RESULTS-sim-phase.md §2: +22% robustness, because the rotor's reflected
  inertia is 2.25× smaller than on 20T). Both pulleys are still bought (20T: ~46 N stall, 1.07 m/s; 30T: ~31 N, 1.60 m/s. Calculated from
  specs, not measured).
- **Joint 1:** an 8 mm shaft through **two 608 bearings** in the cart. Two bearings stop the shaft tilting;
  one lets it bind and misaligns the sensor. The diametric magnet is glued to the shaft end, and the
  **AS5047P is fixed to the cart 1–1.5 mm from it**. The sensor doesn't rotate, so **the rod can spin
  continuously for swing-up with no slip ring**.
- **Rod 1:** **30 cm aluminium tube** (falls in about 0.14 s, a moderate middle ground). Spare tube allows a
  40 cm rod if needed.
- **Joint 2 (double only):** mounted on top of rod 1, with smaller and lighter bearings. The second AS5047P
  rides on rod 1, and flexible wires run along it. That's fine because the double only balances, so the joint
  moves through a limited arc. The top rod is carbon fibre. **Rod lengths and which goes at the bottom are
  decided in sim.** Quanser's lab double uses 33.65 cm and 20.0 cm links, but doesn't document which is at the
  bottom.
- **Tapper:** a servo swings a printed hammer into the rod, for repeatable physical pushes (§12).

## 8. Electronics and safety wiring

- **Power path (revised 2026-09-25; full diagram in hardware/electronics.md):** PSU (12 V) → **7.5 A fuse**
  → **latching E-stop** → **Schottky diode** → bus with a **2200 µF capacitor + TVS clamp** → Cytron MDD10A (ordered in place of the MD13S).
  The diode stops braking energy reaching the PSU, whose over-voltage protection (12.6–16.2 V) latches off
  until re-powered.
- **Brake relay in the motor lead (revised 2026-09-25):** a 5-pin changeover automotive relay connects the
  motor to the driver when energised and **shorts the motor when it drops**. The sim shows cutting power lets
  the cart coast 720–840 mm from top speed (into the rail end); shorting the motor stops it in ~110 mm.
- **Relay coil loop:** 12 V after the E-stop → **end-stop switches (normally closed, in series)** → relay coil
  → ground, with a **flyback diode across the coil**. The E-stop, either end-stop, or a power failure drops
  the relay and brakes the motor **independently of any software**; the micro switches only carry coil
  current. End-stops trigger at about ±350 mm, with printed TPU bumpers at the rail ends.
- **Logic:** the Pico 2 is powered by USB from the Mac. PWM + DIR go to the Cytron. The motor encoder A/B lines
  go to the Pico, decoded by PIO. The AS5047P connects over SPI. Every ground is shared.
- **Also:** the end-stop states are read by the Pico so it can report *why* it stopped. The switches are
  SPDT; how the Pico reads them without mixing 12 V into a 3.3 V pin (a spare contact, a divider or an
  optocoupler) is decided in plan step 2.1.
- **Cytron MD13S (verified in PLAN 2.1 from the user manual v1.1):** logic HIGH is 3.0–5.5 V, so the Pico's
  3.3 V works with 0.3 V of margin; PWM is up to 20 kHz; its built-in current limit is 30 A, far above our
  motor, so it gives no protection at our currents. **The MDD10A ordered instead (manual v1.1, May 2015)** has
  the same logic levels, 20 kHz limit and truth table (PWM LOW → both outputs LOW), 10 A continuous / 30 A peak
  per channel, 5–30 V on Rev 2.0 (check the board is marked REV2.0; Rev 1.0 was 25 V), and no stated current
  limit, which changes nothing here for the same reason.
- **Braking energy (decided 2026-09-25):** diode + 2200 µF + TVS, sized from the sim (worst burst ~0.16 J;
  bus peaks ≈ 17.8 V, under the driver's 30 V). The Pico's PWM and DIR pins get 10 kΩ pull-downs so the motor
  is braked (MD13S and MDD10A: PWM LOW = outputs LOW) while the Pico boots or is unplugged. Logic level: direct
  connection, measured ≥ 3.0 V before any motor power (the RP2350 guarantees only 2.62 V HIGH).

## 9. Firmware (Pico 2, Arduino-style C++ on the arduino-pico core)

**Responsibilities:**
1. **Fixed-rate control loop.** Target ~500 Hz for the single and ~1 kHz for the double. **Final rates come
   from the sim phase.** The AI policy decides at **50–100 Hz** (the evidence-supported range for real
   cart-pole RL), with its output held between decisions.
2. **Sensing:** motor encoder via PIO (starting from Raspberry Pi's `pico-examples` quadrature encoder);
   AS5047P via SPI; the velocities are estimated.
3. **Controller modes:**
   - `IDLE`
   - `CLASSICAL`: energy-pumping swing-up → LQR once near upright (hand-over rule in §10)
   - `AI`: swing-up policy → balance policy (hand-over rule in §10)
   - `LQR_ONLY`: for started-upright tests and the double
   - `SYSID`: plays prerecorded motor voltage sequences and logs the response
   - `KICK`: applies a software disturbance of a set size
4. **NN inference:** a small MLP (size fixed in the plan, e.g. 2 × 64). The weights are sent from the Mac and
   stored on the Pico. Written as plain matrix multiplication.
5. **Safety, on the Pico and not the Mac:**
   - **Heartbeat watchdog:** the motor is off if no valid heartbeat arrives from the Mac within **100 ms**
     (the WPILib motor-safety default).
   - **Hardware watchdog:** resets the Pico if the firmware hangs, and the motor is off on boot.
   - **Soft travel limits:** the motor is off when the cart nears the rail ends.
   - **Fall detection:** the motor is off in balance modes when the angle goes beyond the recoverable limit.
   - **Output clamp:** a maximum duty cycle and a maximum rate of change.
6. **Telemetry:** timestamp, cart position and velocity, angle(s) and velocities, motor command, mode, and
   fault flags, streamed to the Mac.

## 10. Mac-side software (Python; long training on the Windows PC)

- **Simulator:** **MuJoCo**, starting from Gymnasium's `InvertedPendulum-v5` and `InvertedDoublePendulum-v5`
  models, which are cart-based. They're edited to match the real rig's masses, lengths, rail limits and motor.
  The stock tasks are balance-only, so **swing-up needs a custom environment and reward**. A hand-written
  simulator is optional later, as a cross-check.
- **Motor model:** a DC motor model, first from the Pololu datasheet (stall torque, no-load speed, stall
  current), then **fitted to recordings from the real rig**. That's system identification, done in `SYSID`
  mode.
- **Classical control:**
  - LQR designed with `python-control` or `scipy.linalg.solve_continuous_are`, from a model linearised at the
    upright position (sympy or MuJoCo can supply it).
  - Energy-pumping swing-up following Tedrake's *Underactuated Robotics* (MIT).
  - Hand-over to LQR inside its region of attraction.
- **AI:** **Stable-Baselines3, SAC**, running on CPU (SB3's own guidance for small MLP policies).
  - **Two policies:** swing-up and balance, with a hand-over, following Xu & Tran (June 2026), who did zero-shot
    sim-to-real swing-up this way on a real cart-pole.
  - **Sim-to-real techniques, all required:**
    - **Domain randomisation** of masses, friction, motor constants, gearbox play and sensor noise.
    - A **first-order action-smoothing filter**, also applied on the Pico. It prevents bang-bang
      actuation, the widely reported failure mode.
    - **Modelled control delay.**
    - **The motor model fitted from system ID.**
  - Training is expected to take **hours to days for swing-up**. Balance is quick.
- **Communication with the Pico:** USB CDC serial, **binary packets with COBS framing and a checksum**.
  Messages: telemetry, commands/mode, parameter upload (gains, NN weights), heartbeat.
  **Measure the round-trip latency early**; no published figure exists for the RP2350 over USB CDC.
- **Test runner:** runs the §12 test set automatically, then logs every trial and computes the metrics.
- **Visualisation:** **PlotJuggler** for live tuning. A **React project page** is the presentation (§13).

## 11. The simulation phase (first 3–5 days, before ordering the motor)

The sim is built from the motor's datasheet. It answers, in this order:
1. **Is the Pololu 37D 6.3:1 strong and fast enough** to swing up a 30 cm aluminium rod within ~1 m of rail?
   → If yes, **order the motor**.
2. **20T or 30T pulley?**
3. **Is 30 cm a sensible length**, or does 40 cm clearly help?
4. **What loop rates are needed** for the single, and for the double?
5. **For the double: which rod lengths, and which is at the bottom?**
6. **Is the 12 V 5 A PSU enough?** Log the supply current through swing-up and balance (peak, RMS and time
   above 5.25 A), and check whether swing-up still succeeds with the supply current capped at ~5 A. If yes,
   buy the GST60A12; if not, buy a larger 12 V unit and resize the fuse. **Ordered with the motor.**

**Time-box rule:** if an answer isn't clear by day 5, use the default (Pololu, 20T, 30 cm) and order anyway.
The sim phase must not eat into hardware time. A datasheet-only sim is optimistic: it can show "clearly won't
work" early, but only real hardware can confirm "works".

## 12. Test protocol (fixed before any real results)

Both controllers run on the **same Pico, same rig, same day, same rod**. Each runs at its designed rate, and
**the rates are reported openly**. Trials alternate between controllers so that drift (belt tension,
temperature) affects both equally.

**Disturbances:**
- **Software kicks** (most trials): a fixed cart jolt, in a ladder of increasing sizes.
- **Physical tapper** (the physical test and the video): the servo hammer strikes the rod. Tap consistency
  is checked before use.

**Core metrics:**

| Metric | Definition | Trials |
|---|---|---|
| **Swing-up success rate** | Hanging at rest → balanced within the time limit, and held for the hold time | **20 per controller** |
| **Settling time** | After a standard kick, the time until the angle stays within the settle band | 20 per controller |
| **Largest push survived** | The biggest rung on the kick ladder recovered in **≥ 8 of 10** attempts | 10 per rung |
| **Motor effort** | Mean absolute (or RMS) motor command during settling | From the trials above |

**Numbers to fix before the first hardware trial:** time limit, hold time, settle band, and the kick ladder's
sizes. Choose them from the sim; they then **may not change** after real results are seen.

**Honest reporting:** report every trial, including failures, plus the loop rates, the sim-to-real steps
used, and anything that was tuned on hardware.

## 13. Presentation deliverables

- **Two videos:**
  - **~60 s highlight, results first:** swing-up, a push recovery, the scorecard.
  - **Longer technical cut:** the build, how each controller works, sim-to-real, the test results, and what's
    next (the double/triple).
- **Headline chart: a scorecard** of the four core metrics, classical vs AI side by side.
- **Public GitHub repo** with a strong README: the story, the scorecard, embedded clips, how to build it, and a
  bill of materials.
- **A React project page:** build its skeleton early (e.g. during overnight training runs), with empty slots
  for results and clips. Fill it in during 2–5 Nov.
- **Film and write as you go:** every milestone gets filmed, with a paragraph written the same day.

## 14. Timeline (milestones; the plan breaks these into days)

| Dates | Milestone |
|---|---|
| **Thu 24 – Mon 28 Sep** | Order everything except the motor and PSU. Sim phase (§11): MuJoCo model, datasheet motor model, answers 1–6. Learn the basics: Brunton's *Control Bootcamp*, Python. **Order the motor and PSU** by about 28 Sep |
| Parts arrive, ~26–30 Sep | Print the cart, end blocks and joint housings. Pico bring-up: blink → AS5047P over SPI → motor encoder over PIO → USB telemetry → **measure the latency** |
| ~1–7 Oct | Motor arrives. Assemble the rig. **Safety first:** fuse, E-stop, end-stops, watchdog. Run system ID and fit the sim |
| ~8–14 Oct | LQR balance (started upright) → energy swing-up → **classical complete**. Train the AI in the fitted sim |
| ~15–21 Oct | AI balance on hardware → AI swing-up + hand-over. Build the tapper. **Run the full test set.** → **Single DONE** (target ~21 Oct) |
| ~22 Oct – **1 Nov** | **Double (stretch):** sim the rod layout → buy the 2nd AS5047P + carbon rod → build → LQR balance. **Hard stop 1 Nov** |
| **2 – 5 Nov** | Edit both videos, finish the React page and README, submit |

## 15. Risks

| Risk | Mitigation |
|---|---|
| Sim-to-real gap: the AI works in sim, fails on the rig | System ID, domain randomisation, action smoothing, delay modelling (§10). **Classical is the fallback result**: the comparison still stands if the AI underperforms, and the finding is still honest |
| Swing-up training takes days | Start training as soon as the fitted sim exists. Use the Windows PC for long runs. Train balance first (quick) |
| Motor too weak or slow | Checked in the sim phase **before** buying. The 20T/30T pulley swap trades force for speed |
| Gearbox play (~1–2°) | Measure it with the encoders. Model it in domain randomisation. It matters most for the double |
| Sensor noise from the motor | Short, separated sensor wiring. The SPI sensor is chosen partly for this. Filtering |
| USB latency or jitter | Control runs on the Pico; the Mac isn't in the loop. Measure latency on day 1 of bring-up |
| Pico control-loop timing | PIO for the encoder; keep the NN small and **benchmark its inference time** early |
| Parts delay or stock-outs | UK stock only. Alternatives noted (RS for the PSU, RobotBits for the driver) |
| Budget overrun (~£11–46 over) | §6 savings; skip anything already owned |
| The double stalls | Gated behind a finished single, with a hard stop on 1 Nov. The single alone is a complete result |
| Presentation rushed | Film and write as you go; build the React page skeleton early; 2–5 Nov protected |
| Safety: fast cart, pinch points | Hardware E-stop and end-stops cut power independently of software, plus the Pico watchdog, soft limits, fall detection and output clamps. Keep hands clear during runs |

## 16. Roads not taken (and when to revisit)

| Rejected | Why | Revisit if |
|---|---|---|
| Other projects (room sensor, reaction-wheel cube, wind tunnel, …) | The room sensor needs a pilot site and is a mature market; the cube's fallback (edge balance) is a beginner build | This project is abandoned |
| Rotary (Furuta) pendulum | A triple is barely documented; the cart matches the goal | Space or budget force it |
| Open-loop stepper | Thin evidence (one jerky single, no doubles); missed steps | Never for this build |
| Closed-loop stepper | £90–130, no pendulum precedent | – |
| BLDC + ODrive | Over budget; hardest setup | Going for the triple after the deadline |
| JGB37-520 motor | Unknown gearbox play and variable quality (it was the cheaper fallback) | The Pololu is unavailable |
| V-slot / steel rods | Friction (ZIPY) / sag at 1 m | – |
| MT6701 | No UK-stock SSI board; the China listing arrives 22–28 Oct | A UK SSI board appears |
| AS5600 | I2C, 0.088°, motor-noise report | Budget emergency, single only |
| Mac in the loop | Unmeasured latency and jitter; risky for the double | Early quick experiments only |
| JavaScript on the Pico (Kaluma) | Interpreted, GC pauses, no control-loop evidence. DeviceScript is archived | – |
| MicroPython | GC jitter, no 1 kHz evidence | – |
| PPO / CleanRL | Chose SAC: sample-efficient, suits continuous control and hardware fine-tuning | SAC won't train |
| One policy for swing-up + balance | Less evidence on hardware; two policies mirror the classical structure | Two policies can't hand over cleanly |
| PC (ATX) power supply | In a working PC, needed for weeks, small saving | – |
| Buy everything after the sim | Loses ~a week of parallel build time | – |
| End-stops in series with motor power | Micro switches have no usable DC power rating (Omron V-15: 0.6 A at 125 V DC); DC arcs can weld them shut | Never; use the relay |
| DC-rated micro switches (Omron V-21(IN), 12 A at 30 V DC) | Hard to source in the UK, only 12 A of headroom | The relay approach fails |
| Upsizing the PSU before the sim | Adds cost before there's evidence it's needed | The sim shows swing-up needs more than ~5 A |

## 17. Evidence and sources

- Double pendulum cart (DC servo, V-slot, 5000-line encoders, STM32, 2 kHz): https://hackaday.io/project/12827-double-pendulum-cart
- ZIPY cart-pole (DC motor + belt, V-slot friction fix, ~61 cm rod): https://blog.benwiener.com/programming/2018/05/10/cart-pole.html
- Open-loop stepper cart (jerky motion): https://hackaday.io/project/181812-inverted-pendulum-on-a-cart
- Single LQR cart with AS5600: https://github.com/GabrielRhys/Single-Inverted-Pendulum-w-LQR
- Double on MGN12: https://github.com/RunTheBot/Inverted-pendulum
- zjor cart (Arduino + Python): https://www.hackster.io/zjor/inverted-pendulum-on-a-cart-199d6f
- Utah double (never achieved stable control): https://amir-yazdani.github.io/project/inv_pendulum/
- DIVA double inverted pendulum thesis (Maxon DC motor): https://www.diva-portal.org/smash/get/diva2:827020/FULLTEXT01.pdf
- TU Wien triple swing-up (linear motor, backlash-free), Glück, Eder & Kugi 2013: https://www.acin.tuwien.ac.at/file/publications/cds/pre_post_print/glueck2013.pdf
- Multi-arm pendulum on a cart benchmark: https://arxiv.org/abs/2205.06231
- Xu & Tran 2026, zero-shot sim-to-real cart-pole swing-up (action smoothing, domain randomisation, two policies): https://arxiv.org/abs/2606.22145
- ferrolho rotary pendulum (MuJoCo → hardware, 689-param NN on an Arduino Nano): https://github.com/ferrolho/rotary-inverted-pendulum
- Re-Forward (RL on STM32): https://github.com/kartman88/Re-Forward
- Gymnasium MuJoCo InvertedPendulum / InvertedDoublePendulum: https://gymnasium.farama.org/environments/mujoco/inverted_pendulum/
- Stable-Baselines3 PPO/SAC docs (CPU guidance): https://stable-baselines3.readthedocs.io/
- Tedrake, *Underactuated Robotics* (energy swing-up + LQR): https://underactuated.mit.edu/
- Pololu 37D 6.3:1 12 V with encoder: https://www.pololu.com/product/4757 · backlash forum: https://forum.pololu.com/t/backlash-37d-gear-motor-with-19-1-ratio/7777
- Quanser linear double inverted pendulum (33.65 cm / 20.0 cm links): https://www.quanser.com/products/linear-double-inverted-pendulum/
- Arduino-Pico core (RP2350 support): https://github.com/earlephilhower/arduino-pico
- Pico PIO quadrature encoder: https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio
- SimpleFOC MT6701 driver (I2C too slow for control): https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6701/README.md
- AS5047P adapter kit (magnet included): https://uk.farnell.com/ams-osram-group/as5047p-ts-ek-ab/adapter-board-kit-position-sensor/dp/4158644
- Mean Well GST60A series spec (overload 105–150%, hiccup mode): https://www.meanwell.com/Upload/PDF/GST60A/GST60A-SPEC.PDF
- Omron V-series micro switch datasheet (V-15: 0.6 A at 125 V DC; V-21(IN): 12 A at 30 V DC): https://omronfs.omron.com/en_US/ecb/products/pdf/en-v.pdf
- Cytron MD13S user manual (logic HIGH 3–5.5 V, 20 kHz, 30 A current limit): https://www.farnell.com/datasheets/3759023.pdf
- WPILib motor safety (100 ms watchdog): https://docs.wpilib.org/en/2023_a/docs/software/hardware-apis/motors/wpi-drive-classes.html
- COBS framing: https://blog.mbedded.ninja/programming/serialization-formats/consistent-overhead-byte-stuffing/
- DeviceScript archived: https://github.com/microsoft/devicescript · Kaluma: https://kalumajs.org/

## 18. Open items for the plan

- Confirm ⚠️ prices at checkout: rail at 1000 mm, AS5047P kit, MD13S VAT, E-stop, shaft, relay.
- How the Pico reads the end-stop states (plan 2.1), and the PSU/fuse size (sim Q6).
- The Cytron MD13S's 3.3 V input thresholds, and the regenerative-braking mitigation (datasheets).
- NN size and its inference-time benchmark on the RP2350.
- The hand-over rules (classical and AI): the energy and angle thresholds, from the sim.
- The test-protocol numbers (§12), fixed from the sim before the first hardware trial.
- Git: initialise this folder as a repo (public at the end), with folders `sim/`, `firmware/`, `analysis/`,
  `hardware/` (CAD/STL), `web/` (React page), `media/`.
