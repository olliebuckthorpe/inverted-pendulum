# Bench bring-up: from parcels to a safe rig (PLAN 2.3, 2.4, 2.6, 3.1–3.4)

One stage at a time. **Each stage adds one thing and proves it before the next gets power.** Wiring and pin
numbers are in `hardware/electronics.md`; mechanics in `hardware/cad/LAYOUT.md`; parts in `hardware/orders.md`.
Mac commands run from `sim/`; `PORT` is the Pico's port (`ls /dev/cu.usbmodem*`).

**Rules for every stage:** E-stop within reach before any 12 V; the belt is **off** until stage 6; hands off the
belt and cart whenever the motor can move; disconnect 12 V before re-wiring.

## Stage 0 — Unpack and check (no power)

Tick the "Checks on arrival" list in `hardware/orders.md`: rail smoothness, MDD10A marked **REV2.0**, relay pin
**87a**, micro-switch hole size and roller height, M2/M2.5 screw lengths. Weigh the printed cart parts, carriage,
bearings and shaft on kitchen scales and update `sim/params.yaml` `cart.mass` (currently the CAD estimate).

## Stage 1 — Pico alone on USB (PLAN 2.3)

1. **Blink** (the Pico 2 W's LED is on its wireless chip, so this one sketch is built for `rpipico2w`). Hold
   BOOTSEL while plugging in the USB lead, then:
   `arduino-cli compile --fqbn rp2040:rp2040:rpipico2w firmware/bringup/blink_w -u -p PORT`
   **Done when** the LED blinks and `arduino-cli monitor -p PORT` prints `blink 1, 2, 3…`.
2. **Real firmware:** `firmware/build.sh --upload -p PORT`, then `uv run python bringup.py PORT watch`.
   Expect mode IDLE, duty +0.00, and (nothing wired yet) ✗ on every live input and the E-stop fault latched.
   That is correct: with no 12 V the Pico reads "E-stop pressed".

## Stage 2 — Angle sensor (PLAN 2.4)

1. **Read the AS5047P-TS_EK_AB operation manual first** and set the board for **3.3 V** (VDD and VREG tied).
   Don't power it until that's confirmed (electronics.md ⚠️).
2. Wire it to the Pico: 3V3(OUT) → VDD, GND, GP16 ← MISO, GP17 → CSn, GP18 → CLK, GP19 → MOSI. Magnet in its
   printed cap, centred on the chip at the CAD's gap.
3. `bringup.py PORT watch`: **✓ angle read OK**; turn the shaft slowly by hand: θ runs smoothly through a full
   turn with no jumps.
4. `bringup.py PORT calibrate` with the rod hanging still: saves the angle's zero (upright = hanging + 180°) to
   `hardware/rig-calibration.json`; every later command and `pico_setup.py` use it. Commit that file.
5. `bringup.py PORT noise` with the rod hanging still: appends the std-dev to `analysis/sensor-noise.md`. Put
   it into `sim/params.yaml` `sensing.angle_noise_deg`. **2.4 done.**

## Stage 3 — Motor encoder, turned by hand (PLAN 3.3, first half)

Motor leads (red/black) **not connected to anything**. Encoder: yellow → 1 kΩ → GP2, white → 1 kΩ → GP3,
blue → VBUS, green → GND. `watch`: turn the motor shaft by hand; x changes smoothly, and the sign flips with
direction. (The distance check with a ruler comes in stage 6, once the cart is on the belt.)

## Stage 4 — 12 V logic side, no driver, no motor (PLAN 3.2, part 1)

1. PSU → 7.5 A fuse → E-stop (NC contacts) → node **P12**. Relay coil loop: P12 → left end-stop (NC) → **N1** →
   right end-stop (NC) → **N2** → relay coil (85/86) → GND, **1N4001 across the coil, band to the +12 V side**.
2. Before connecting the dividers to the Pico, **measure each divider output: ≈ 2.98 V, never above 3.3 V.**
   Then GP10 ← P12 divider, GP11 ← N1 divider, GP12 ← N2 divider; common GND to the Pico.
3. `watch` with 12 V on: ✓ P12, ✓ N1, ✓ N2, relay clicked in. Press the E-stop: all three ✗, relay drops.
   Release. Press each end-stop by hand: the pattern matches the table in electronics.md and the relay drops.

## Stage 5 — Driver and bus, motor still disconnected (PLAN 3.2, part 2)

1. After the E-stop: **MBR1045G** (band towards the bus) → **BUS**; **2200 µF** across BUS–GND (**stripe = −**);
   **1.5KE20A** across BUS–GND (**band to BUS**); MDD10A **B+** ← BUS, **B−** ← GND.
2. Logic to the MDD10A 5-way connector: GND, PWM1 ← GP14, DIR1 ← GP15, each with its **10 kΩ pull-down**;
   PWM2 and DIR2 to GND (channel 2 unused).
3. **Logic levels:** `bringup.py PORT levels` holds PWM, then DIR, steadily HIGH for 8 s each. **Measure ≥ 3.0 V**
   at the MDD10A connector each time (electronics.md). If lower, stop: add a 74AHCT125 buffer.
4. `bringup.py PORT spin --duty 0.1 --ms 1000` and `--duty -0.1`: the MDD10A's M1A / M1B LEDs light in turn.

## Stage 6 — Motor on, belt off, then the rig (PLAN 3.1, 3.2 part 3, 3.3)

1. Motor through the brake relay: **30 ← motor red, 87 ← MDD10A M1A, 87a ← motor black and MDD10A M1B**.
   Motor clamped on the bench, pulley on, **no belt**. `spin --duty 0.1` and `-0.1`: turns both ways.
   `spin --duty 0.15 --ms 800`, and press the E-stop mid-spin: it stops **sharply** (braked), not coasting.
   (With no belt the encoder still counts "cart" travel, so the soft limit ends longer spins by itself.)
2. Mechanical assembly per `hardware/cad/LAYOUT.md` (PLAN 3.1): beam clamped on the desk edge, rail, end blocks,
   belt tensioned (firm, no slack), cart, joint, rod. The cart slides the full rail by hand without binding.
3. `bringup.py PORT ruler`: move the cart ≥ 500 mm along a steel rule. **Within 1 %: PLAN 3.3 done.**
4. `bringup.py PORT loop --minutes 10`: **max exec < 1000 µs: PLAN 2.6 done** (appends `analysis/loop-timing.md`).

## Stage 7 — Safety checklist 🚦 (PLAN 3.4)

Work through `hardware/safety-checklist.md`, filling in the date and result of each test.
**No closed-loop control (LQR, swing-up, AI) until every line is ticked.**
