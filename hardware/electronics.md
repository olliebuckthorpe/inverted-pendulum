# Electronics (PLAN 2.1) — power path, pin map, decisions

Written Fri 25 Sep 2026, before any wiring. **Updated Wed 30 Sep for the parts actually ordered** (see
`hardware/orders.md`): **Cytron MDD10A** instead of the MD13S (not obtainable in time on 29 Sep: The Pi Hut
sold out, Active Robots and RobotBits sites down, Farnell 16 weeks, Amazon arriving 16–31 Oct), **1.5KE20A** TVS instead of 1.5KE22A (RS out of stock), **1N4001** instead of
1N4007 (same diode, 50 V grade, fine across a 12 V coil), **Pico 2 W** instead of Pico 2 H (cheaper in the
one Amazon basket; the firmware still builds for the plain Pico 2, see `firmware/build.sh`). Every figure below is quoted from a datasheet (sources at the end)
or measured in the sim (`sim/`). Decisions were made by Ollie from the options and evidence listed.

## 1. Power path

**Revised 25 Sep (brake relay):** the relay no longer cuts the driver's supply; it sits in the **motor lead**
and **shorts the motor when it drops**. Sim, from top speed 1.6 m/s: cutting power lets the cart coast
**720–840 mm** (into the rail end); shorting the motor stops it in **100–118 mm** (the motor becomes a
generator driving current through its own windings, which brakes it).

```
 Mean Well   ┌───────┐   ┌─────────┐      P12
 GST60A12 +──┤ 7.5 A ├───┤ E-stop  ├───────┬──────────────────────────────────────────────┐
 12 V 5 A    │ fuse  │   │latching │       │                                              │
             └───────┘   │ NC      │       │  relay coil loop (signal current only)       │
                         └─────────┘       ├──┤end-stop L (NC)├─N1─┤end-stop R (NC)├─N2─┤relay coil├── GND
                                           │                                (1N4001 across the coil)
                                           ▼  Schottky ≥ 8 A, ≥ 40 V (energy can't flow back to the PSU)
                                           │
                        BUS ───────────────┼────────────┬───────────────┐
                                           │            │               │
                                     2200 µF 35 V   TVS 1.5KE20A    MDD10A B+ (B− → GND)
                                           │            │               │
 GND ──────────────────────────────────────┴────────────┴───────────────┴── common with the Pico's GND

 MDD10A M1A ─────────── relay 87  (NO)          relay: 12 V automotive CHANGEOVER, 5-pin (30 / 87 / 87a)
                                   ╲
                         relay 30 (COM) ──────── motor lead RED
                                   ╱
 motor lead BLACK ──┬── relay 87a (NC)          energised (all clear):  30–87  → motor driven by the MDD10A
                    │                           dropped (any trip):     30–87a → motor leads shorted = braking
 MDD10A M1B ─────────┘   (channel 2, M2A/M2B, unused)
```

- **Fuse first**, then the **latching E-stop**, then the split: the relay coil loop and the driver supply.
- **Pressing the E-stop** removes 12 V from both the driver and the relay coil, so the relay drops and shorts
  the motor. **Hitting an end-stop** drops the relay the same way. No software is involved (SPEC §8), and a
  power failure also leaves the motor shorted, braked (fail-safe).
- **End-stops only carry the relay coil's current** (micro switches have no usable DC power rating: Omron
  V-15 is 0.6 A at 125 V DC). The relay's contacts carry the motor and braking current (~5.3 A shorted at
  1.6 m/s: 11.6 V / 2.18 Ω), well inside a 30–40 A automotive relay.
- **End-stop position: trigger at X ≈ ±350 mm** (cart centre), so the ~118 mm braking distance fits before the
  carriage reaches the rail end (~±477 mm). The controllers use at most ±325 mm. **Printed TPU bumpers** at the
  rail ends absorb anything left over.
- **Firmware adds a speed-aware soft limit:** brake early if |x| + v²/(2·a_brake) would pass the soft limit.
- **Schottky diode + capacitor + TVS (braking energy), decided 25 Sep:** braking pushes energy back into the
  supply. The GST60A12's over-voltage protection is 12.6–16.2 V and **latches off until re-powered**. In the
  sim, the worst 50 ms of returned energy is 164 mJ (single swing-up) and 146 mJ (double tour); peak reverse
  current 0.31–0.51 A. Keeping the PSU under 12.6 V without a diode would need ~27,000 µF. With the diode, the
  PSU never sees it; 2200 µF absorbs a 0.2 J burst with the bus rising to √(11.6² + 2·0.2/0.0022) ≈ 17.8 V,
  under the MDD10A's 30 V maximum (Rev 2.0; the v1.1 manual's table says 25 V for Rev 1.0, so **check the board
  is marked REV2.0**). The TVS (1.5KE20A: 17.1 V stand-off, 19.0–21.0 V breakdown) is a backstop clamp. The
  17.8 V sim peak sits between its stand-off and breakdown, so it may leak a little during a braking burst,
  which only helps; in normal running (≤ 12 V) it does nothing. At
  these currents (≤ 0.5 A reverse, 0.2 J) it holds the bus near its breakdown voltage, far below 25 V. The diode costs ~0.4 V;
  the sim still swings up 10/10 (single) and completes the double tour 5/5 at 11.6 V.
- **Firmware current limiter** (SPEC §11 Q6): supply current ≤ 4.5 A until Phase 4 fits R and Ke, then 5 A.

## 2. Logic wiring and pin map (Raspberry Pi Pico 2 W, run as a Pico 2)

The ordered board is a **Pico 2 W**: same RP2350 and the same GP0–GP22 and GP26–28 as the Pico 2. On the W,
GP23/24/25/29 belong to the wireless chip (power-enable, data, chip-select, clock). The firmware is built for
the plain Pico 2 (`firmware/build.sh`), so it never drives GP23 and the radio stays unpowered; the only loss is
the on-board LED, which the firmware doesn't use.

All the RP2350's digital GPIOs are **fault tolerant**: they accept up to 5.5 V *while the Pico is powered*
(IOVDD = 3.3 V), and start up with pull-downs. The ADC pins GP26–29 are **not** 5 V tolerant; keep them for
3.3 V signals only.

| Pico pin | Signal | Connects to | Notes |
|---|---|---|---|
| GP2 | Motor encoder A | Pololu yellow, via **1 kΩ series** | PIO quadrature needs A and B on consecutive pins |
| GP3 | Motor encoder B | Pololu white, via **1 kΩ series** | |
| VBUS (5 V) | Encoder Vcc | Pololu blue | Encoder needs 3.5–20 V, so 3.3 V is too low. VBUS goes up and down with the Pico's USB power; the 1 kΩ resistors limit current in the instant before the Pico's 3.3 V rail is up |
| GND | Encoder GND | Pololu green | |
| GP14 | Motor PWM | MDD10A **PWM1**, **10 kΩ pull-down to GND** | 20 kHz, sign-magnitude. PWM LOW = both outputs LOW = motor braked (MDD10A truth table, same as the MD13S). The pull-down keeps it braked while the Pico is booting or unplugged (the manual doesn't state an internal pull-down) |
| GP15 | Motor DIR | MDD10A **DIR1**, **10 kΩ pull-down to GND** | 5-way 2510 connector: GND, PWM2, DIR2, PWM1, DIR1 |
| GND | Logic ground | MDD10A GND (connector pin 1) | Shared ground (SPEC §8). PWM2/DIR2 unused: tie to GND |
| GP16 | SPI0 RX (MISO) | AS5047P #1 MISO (and #2 for the double) | The chip runs at 3.3 V with VDD and VREG tied (datasheet). ⚠️ **Not yet checked: how the AS5047P-TS_EK_AB adapter board is set up for 3.3 V vs 5 V** (read its operation manual on arrival, before powering it) |
| GP17 | SPI0 CSn #1 | AS5047P #1 CSn (joint 1) | |
| GP18 | SPI0 SCK | AS5047P #1/#2 CLK | Up to 10 MHz (AS5047P datasheet); start at 1 MHz over the joint-2 cable |
| GP19 | SPI0 TX (MOSI) | AS5047P #1/#2 MOSI | |
| GP20 | SPI0 CSn #2 | AS5047P #2 CSn (joint 2, double only) | Runs through the flexible joint-1 cable loop (PLAN 2.5b) |
| 3V3(OUT) | Sensor power | AS5047P VDD+VREG | |
| GP10 | Sense P12 | divider 10 kΩ / 3.3 kΩ from node P12 | HIGH = E-stop released |
| GP11 | Sense N1 | divider from node N1 | HIGH = left end-stop closed |
| GP12 | Sense N2 | divider from node N2 | HIGH = right end-stop closed |
| GP13 | Tapper servo | servo signal (PLAN 7.1) | Servo powered separately at 5 V, shared ground |
| ~~GP25~~ | ~~On-board LED~~ | – | Not available on the Pico 2 W (wireless chip). Status is reported over telemetry instead |

**End-stop sensing (voltage dividers, decided 25 Sep):** each divider gives 12 V × 3.3/(10 + 3.3) ≈ 2.98 V
(Pico logic HIGH is ≥ 2.0 V; each divider draws ~0.9 mA). They're fed from the PSU side of the diode, which
the PSU regulates at 12 V, so they never see the braking spike.

| P12 | N1 | N2 | Meaning |
|---|---|---|---|
| H | H | H | All clear, relay energised (motor connected to the driver) |
| H | L | L | **Left end-stop hit** |
| H | H | L | **Right end-stop hit** |
| L | L | L | **E-stop pressed** (or no 12 V) |

**Logic level (direct + measure, decided 25 Sep):** the MDD10A (like the MD13S) needs ≥ 3.0 V for HIGH; the RP2350 guarantees
only 2.62 V at full rated drive current (2–12 mA), though at the ~0.3 mA a 10 kΩ pull-down draws it sits near
3.3 V, and Cytron markets the MDD10A for 3.3 V logic. **Check before any motor power:** with the PWM pin set
HIGH, measure ≥ 3.0 V at the MDD10A input (and ≤ 0.5 V for LOW). If it reads low, add a 74AHCT125 5 V buffer.

## 3. Safety behaviour this wiring gives (feeds the PLAN 3.4 checklist)

- **Motor off at power-on and during reset:** PWM is pulled LOW, so the MDD10A brakes the motor; the RP2350's
  pins also start with pull-downs.
- **E-stop and end-stops disconnect the motor from the driver and short it (braking) with no software
  involved**, and the Pico can report which one.
- **Braking energy can't trip the PSU** (diode), and the bus is limited by the capacitor and TVS.
- Still to be proven on the bench, not assumed: every item in PLAN 3.4.

## 4. Parts added by this step (ordered 29 Sep, RS)

| Part | As ordered | Qty | Notes |
|---|---|---|---|
| Schottky diode | onsemi MBR1045G, 45 V 10 A, TO-220AC (RS 545-2872) | 5 | Needs a small heatsink or a copper area: ~0.4 V × 5 A peak |
| Electrolytic capacitor | Panasonic EEUFR1V222L, 2200 µF 35 V, low-ESR, 105 °C (RS 739-6831) | 2 | Mind the polarity |
| TVS diode | Vishay 1.5KE20A-E3/54, 1.5 kW unidirectional (RS 902-6398) | 10 | Cathode to BUS |
| Relay flyback diode | Vishay 1N4001-E3/54 (RS 628-8931) | 10 | Cathode to the coil's +12 V side |
| Resistors | RS PRO carbon film 0.25 W 5 %: 10 kΩ (707-7745), 3.3 kΩ (707-7701), 1 kΩ (707-7666) | 10 each | |

## Sources

- Cytron MDD10A user's manual v1.1 (May 2015): 5–30 V (Rev 2.0; the absolute-maximum table still says 25 V),
  10 A continuous / 30 A peak (≤ 10 s) per channel; logic HIGH 3–5.5 V, LOW 0–0.5 V; PWM ≤ 20 kHz; truth table
  PWM LOW → both outputs LOW, PWM HIGH + DIR LOW → A high, DIR HIGH → B high. Unlike the MD13S it states no
  current limit; here the 5 A PSU, the firmware limiter and the 7.5 A fuse bound the current.
  https://www.technobotsonline.com/Datasheets3/1519-011-User.pdf
- Cytron MD13S user manual v1.1 (Jun 2018), the originally specified driver: same logic levels, 20 kHz and
  truth table; 30 A current limit. https://www.farnell.com/datasheets/3759023.pdf
- Mean Well GST60A spec: GST60A12 12 V 5 A; overload 105–150% hiccup; over-voltage 12.6–16.2 V, "shut down
  o/p voltage, re-power on to recover". https://www.meanwell.com/Upload/PDF/GST60A/GST60A-SPEC.PDF
- RP2350 datasheet: VOH ≥ 2.62 V and VOL ≤ 0.5 V at IOVDD 3.3 V; VIH 2.0–5.5 V on fault-tolerant pins while
  IOVDD is powered; digital GPIOs are Digital IO (FT) with reset pull-downs.
  https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- Pololu #4757: encoder Vcc 3.5–20 V; A/B outputs 0 V to Vcc; wires red/black motor, green GND, blue Vcc,
  yellow A, white B. https://www.pololu.com/product/4757
- AS5047P datasheet: SPI up to 10 MHz; 3.3 V operation with VDD and VREG tied.
  https://www1.futureelectronics.com/doc/ams/AS5047P-ATSM.pdf
- Sim: `sim/` regen-energy and 11.6 V checks (commands in the PR for this step).
